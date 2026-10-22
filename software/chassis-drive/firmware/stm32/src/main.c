/* Default build is diagnostic-only; stm32-motion enables the motor outputs. */
#include "stm32f4xx_hal.h"
#include "diagnostic_protocol.h"
#ifdef CD_ENABLE_MOTION
#include "motion_control.h"
#define MOTOR_LEFT_IN1 GPIO_PIN_11
#define MOTOR_LEFT_IN2 GPIO_PIN_12
#define MOTOR_RIGHT_IN1 GPIO_PIN_13
#define MOTOR_RIGHT_IN2 GPIO_PIN_14
#define MOTOR_DIRECTION_PINS (MOTOR_LEFT_IN1|MOTOR_LEFT_IN2|MOTOR_RIGHT_IN1|MOTOR_RIGHT_IN2)
static cm_control_t motion;
static cm_parser_t motion_parser;
static volatile bool motion_ready;
static bool motor_initialized;
static TIM_HandleTypeDef pwm;
static void motor_apply(void);
static void motor_off(void);
#endif

#define RX_CAPACITY 256u
#define RX_MAX_AGE_MS 200u

typedef struct { uint8_t byte; uint32_t received_ms; } rx_item_t;
static UART_HandleTypeDef uart;
static IWDG_HandleTypeDef watchdog;
static uint8_t incoming;
static rx_item_t rx_queue[RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_fault;

static void fatal(void)
{
    __disable_irq();
#ifdef CD_ENABLE_MOTION
    motor_off();
#endif
    /* Outputs are disabled above; once started, IWDG will reset us. */
    for (;;) { }
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

static void clock_init(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
#ifdef CD_ENABLE_MOTION
    /* 48 MHz PLLQ for hardware RNG; CPU and timers stay on 16 MHz HSI. */
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = 16;
    oscillator.PLL.PLLN = 192;
    oscillator.PLL.PLLP = RCC_PLLP_DIV4;
    oscillator.PLL.PLLQ = 4;
#endif
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) fatal();
    RCC_ClkInitTypeDef clocks = {0};
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_0) != HAL_OK) fatal();
}

void HAL_UART_MspInit(UART_HandleTypeDef *handle)
{
    if (handle->Instance != USART2) return;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Pull = GPIO_PULLUP;
    pins.Speed = GPIO_SPEED_FREQ_HIGH;
    pins.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &pins);
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void uart_init(void)
{
    uart.Instance = USART2;
    uart.Init.BaudRate = 115200;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX_RX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart) != HAL_OK) fatal();
    if (HAL_UART_Receive_IT(&uart, &incoming, 1) != HAL_OK) fatal();
}

static void watchdog_init(void)
{
    /* Nominal 1 second, subject to LSI tolerance; not a measured guarantee. */
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_32;
    watchdog.Init.Reload = 999;
#ifndef CD_ENABLE_MOTION
    __HAL_DBGMCU_FREEZE_IWDG();
#else
    __HAL_DBGMCU_UNFREEZE_IWDG();
#endif
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) fatal();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle)
{
    if (handle->Instance != USART2) return;
    uint16_t next = (uint16_t)((rx_head + 1u) % RX_CAPACITY);
    if (next == rx_tail) {
        rx_fault = true;
    } else {
        rx_queue[rx_head].byte = incoming;
        rx_queue[rx_head].received_ms = HAL_GetTick();
        __DMB();
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(&uart, &incoming, 1) != HAL_OK) rx_fault = true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle)
{
    if (handle->Instance == USART2) rx_fault = true;
}

static bool pop_byte(rx_item_t *item)
{
    uint32_t saved_mask = __get_PRIMASK();
    __disable_irq();
    bool available = rx_tail != rx_head && !rx_fault;
    if (available) {
        *item = rx_queue[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RX_CAPACITY);
    }
    __set_PRIMASK(saved_mask);
    return available;
}

static void recover_rx(cd_parser_t *parser)
{
#ifdef CD_ENABLE_MOTION
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    cm_control_disable(&motion, CM_UART_FAULT);
    motor_off();
    cm_parser_reset(&motion_parser);
    __set_PRIMASK(mask);
#endif
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    if (HAL_UART_AbortReceive(&uart) != HAL_OK) fatal();
    __HAL_UART_CLEAR_OREFLAG(&uart);
    rx_tail = rx_head;
    rx_fault = false;
    cd_parser_reset(parser);
    if (HAL_UART_Receive_IT(&uart, &incoming, 1) != HAL_OK) fatal();
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

#ifdef CD_ENABLE_MOTION
static void motor_off(void)
{
    if (!motor_initialized) return;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->EGR = TIM_EGR_UG;
    GPIOB->BSRR = (uint32_t)MOTOR_DIRECTION_PINS << 16;
}

static void motor_safe_pins_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM3_FORCE_RESET();
    __HAL_RCC_TIM3_RELEASE_RESET();
    /* Hold L298N enables low before clock/RNG initialization can fail.
       External ENA/ENB pulldowns are still required during MCU reset. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, MOTOR_DIRECTION_PINS, GPIO_PIN_RESET);
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_6|GPIO_PIN_7;
    pins.Mode = GPIO_MODE_OUTPUT_PP; pins.Pull = GPIO_PULLDOWN; pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &pins);
    pins.Pin = MOTOR_DIRECTION_PINS;
    HAL_GPIO_Init(GPIOB, &pins);
    motor_initialized = true;
    /* PB0 belongs to the board's SPI flash CS; L298N has no STBY input. */
}

static void motor_init(void)
{
    pwm.Instance = TIM3;
    pwm.Init.Prescaler = 0;
    pwm.Init.CounterMode = TIM_COUNTERMODE_UP;
    pwm.Init.Period = 1599; /* 16 MHz / 1600 = 10 kHz. */
    pwm.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    pwm.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&pwm) != HAL_OK) fatal();
    TIM_OC_InitTypeDef channel = {0};
    channel.OCMode = TIM_OCMODE_PWM1; channel.Pulse = 0;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH; channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&pwm, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&pwm, &channel, TIM_CHANNEL_2) != HAL_OK) fatal();
    GPIO_InitTypeDef pins = {0};
    pins.Pin = GPIO_PIN_6|GPIO_PIN_7; pins.Mode = GPIO_MODE_AF_PP;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    pins.Pull = GPIO_PULLDOWN; pins.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &pins);
    if (HAL_TIM_PWM_Start(&pwm, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&pwm, TIM_CHANNEL_2) != HAL_OK) fatal();
    motor_off();
}

static uint32_t boot_random(void)
{
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG_HandleTypeDef rng = {0};
    rng.Instance = RNG;
    uint32_t seed;
    if (HAL_RNG_Init(&rng) != HAL_OK || HAL_RNG_GenerateRandomNumber(&rng, &seed) != HAL_OK) fatal();
    return seed; /* Fresh across resets; not a network authentication secret. */
}

static void motor_apply(void)
{
    if (!(motion.flags & CM_ARMED)) { motor_off(); return; }
    for (unsigned i=0;i<2;++i) {
        const uint16_t a = i ? MOTOR_RIGHT_IN1 : MOTOR_LEFT_IN1;
        const uint16_t b = i ? MOTOR_RIGHT_IN2 : MOTOR_LEFT_IN2;
        int16_t duty = motion.output[i];
        uint32_t set = duty>0 ? a : duty<0 ? b : 0;
        volatile uint32_t *ccr = i ? &TIM3->CCR2 : &TIM3->CCR1;
        if ((GPIOB->ODR & (a|b)) != set) {
            *ccr=0; TIM3->EGR=TIM_EGR_UG; /* Latch zero before changing direction. */
            GPIOB->BSRR=((uint32_t)((a|b)&~set)<<16)|set;
        }
        *ccr=(uint32_t)(duty<0 ? -duty : duty)*1600u/1000u;
    }
}

static bool handle_motion(uint8_t byte, uint32_t received_ms)
{
    cm_request_t request;
    if (!cm_request_feed(&motion_parser,byte,&request)) return false;
    cm_reply_t reply;
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    /* RX can fail after pop_byte() returns. Never apply that queued frame;
       the main loop will flush/rearm RX and require a fresh session. */
    if (rx_fault) {
        cm_control_disable(&motion,CM_UART_FAULT);
        motor_off();
        __set_PRIMASK(mask);
        return true;
    }
    uint32_t now=HAL_GetTick();
    cm_result_t result=cm_control_request(&motion,&request,now);
    if (result==CM_OK && (request.opcode==CM_ARM || request.opcode==CM_DRIVE)) {
        motion.last_command_ms=received_ms;
        cm_control_tick(&motion,now);
    }
    motor_apply();
    cm_control_reply(&motion,&request,now,result,&reply);
    __set_PRIMASK(mask);
    uint8_t packet[CM_REPLY_SIZE];
    cm_encode_reply(&reply,packet);
    if (HAL_UART_Transmit(&uart,packet,sizeof packet,20)!=HAL_OK) fatal();
    return true;
}
#endif

int main(void)
{
    HAL_Init();
#ifdef CD_ENABLE_MOTION
    motor_safe_pins_init();
#endif
    clock_init();
#ifdef CD_ENABLE_MOTION
    motor_init();
    cm_control_init(&motion,boot_random(),HAL_GetTick());
    motion_ready=true;
#endif
    uart_init();
    watchdog_init();
    cd_parser_t parser = {{0}, 0};
    for (;;) {
        if (rx_fault) recover_rx(&parser);
        for (unsigned budget = 0; budget < 64; ++budget) {
            rx_item_t item;
            if (!pop_byte(&item)) break;
            if ((uint32_t)(HAL_GetTick() - item.received_ms) > RX_MAX_AGE_MS) {
                cd_parser_reset(&parser);
#ifdef CD_ENABLE_MOTION
                cm_parser_reset(&motion_parser);
                uint32_t mask=__get_PRIMASK();
                __disable_irq();
                cm_control_disable(&motion,CM_UART_FAULT);
                motor_off();
                __set_PRIMASK(mask);
#endif
                continue;
            }
#ifdef CD_ENABLE_MOTION
            if (handle_motion(item.byte,item.received_ms)) { cd_parser_reset(&parser); continue; }
#endif
            cd_request_t request;
            if (!cd_request_feed(&parser, item.byte, &request)) continue;
#ifdef CD_ENABLE_MOTION
            cm_parser_reset(&motion_parser);
            if (request.opcode==CD_STOP) {
                uint32_t mask=__get_PRIMASK();
                __disable_irq();
                cm_control_disable(&motion,0);
                motor_off();
                __set_PRIMASK(mask);
            }
            cd_reply_t reply = {request.sequence,HAL_GetTick(),CM_FIRMWARE_ID,CD_MOTOR_IMPLEMENTED};
#else
            cd_reply_t reply = {request.sequence, HAL_GetTick(), CD_FIRMWARE_ID,
                                CD_NO_MOTOR_IMPLEMENTATION};
#endif
            uint8_t output[CD_REPLY_SIZE];
            cd_encode_reply(&reply, output);
            if (HAL_UART_Transmit(&uart, output, sizeof output, 20) != HAL_OK) fatal();
        }
        if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) fatal();
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
#ifdef CD_ENABLE_MOTION
    if (motion_ready) {
        if (rx_fault) cm_control_disable(&motion,CM_UART_FAULT);
        cm_control_tick(&motion,HAL_GetTick());
        motor_apply();
    }
#endif
}
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&uart); }
void NMI_Handler(void) { fatal(); }
void HardFault_Handler(void) { fatal(); }
void MemManage_Handler(void) { fatal(); }
void BusFault_Handler(void) { fatal(); }
void UsageFault_Handler(void) { fatal(); }
