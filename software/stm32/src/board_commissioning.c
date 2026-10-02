/* Phase 2 only: no motor PWM timer, encoder timer or servo is enabled. */
#include "stm32f4xx_hal.h"
#include "commissioning_application.h"
#include "uart_rx_queue.h"

#if !defined(ROVER_COMMUNICATION_ONLY) || ROVER_COMMUNICATION_ONLY != 1
#error "This board target must remain communication-only"
#endif

#define TELEMETRY_PERIOD_MS 50U
#define MAX_RX_AGE_MS 100U
#define RX_DRAIN_BUDGET 128U

static UART_HandleTypeDef uart2;
static IWDG_HandleTypeDef watchdog;
static commissioning_application_t application;
static uart_rx_queue_t rx_queue;
static uint8_t rx_byte;
static uint8_t tx_frame[ROVER_TELEMETRY_FRAME_SIZE];
static volatile bool rx_restart_needed;

/* Inspect these through SWD; never mix debug text into the binary UART stream. */
volatile uint32_t commissioning_uart_errors;
volatile uint32_t commissioning_rx_overflows;
volatile uint32_t commissioning_stale_rx_bytes;
volatile uint32_t commissioning_tx_errors;
volatile uint32_t commissioning_fatal_code;

static void disable_motor_outputs(void) {
    /* Set output latches BEFORE changing modes, starting with STBY. A physical
       STBY pull-down is still needed before startup code reaches this point. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;
    GPIOD->BSRR = (uint32_t)GPIO_PIN_2 << 16U;
    GPIOD->MODER = (GPIOD->MODER & ~(3UL << 4U)) | (1UL << 4U);
    GPIOD->OTYPER &= ~(uint32_t)GPIO_PIN_2;
    GPIOD->PUPDR = (GPIOD->PUPDR & ~(3UL << 4U)) | (2UL << 4U);
    GPIOA->BSRR = (uint32_t)(GPIO_PIN_6 | GPIO_PIN_7) << 16U;
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 12U) | (3UL << 14U)))
        | (1UL << 12U) | (1UL << 14U);
    GPIOA->OTYPER &= ~(uint32_t)(GPIO_PIN_6 | GPIO_PIN_7);
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15) << 16U;
    GPIOB->MODER = (GPIOB->MODER & ~0xFF000000UL) | 0x55000000UL;
    GPIOB->OTYPER &= ~(uint32_t)(GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    __DSB();
}

static _Noreturn void fatal(uint32_t code) {
    __disable_irq();
    commissioning_fatal_code = code;
    disable_motor_outputs();
    for (;;) {
        __NOP(); /* An already-started independent watchdog will reset us. */
    }
}

static void configure_clock(void) {
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        fatal(2U);
    }
    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV1;
    clocks.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_0) != HAL_OK) {
        fatal(3U);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *handle) {
    if (handle->Instance == USART2) {
        GPIO_InitTypeDef pins = {0};
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        pins.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        pins.Mode = GPIO_MODE_AF_PP;
        pins.Pull = GPIO_PULLUP;
        pins.Speed = GPIO_SPEED_FREQ_LOW;
        pins.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &pins);
        HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

static void configure_uart(void) {
    uart2.Instance = USART2;
    uart2.Init.BaudRate = 115200U;
    uart2.Init.WordLength = UART_WORDLENGTH_8B;
    uart2.Init.StopBits = UART_STOPBITS_1;
    uart2.Init.Parity = UART_PARITY_NONE;
    uart2.Init.Mode = UART_MODE_TX_RX;
    uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart2) != HAL_OK
        || HAL_UART_Receive_IT(&uart2, &rx_byte, 1U) != HAL_OK) {
        fatal(4U);
    }
}

static void configure_watchdog(void) {
    watchdog.Instance = IWDG;
    watchdog.Init.Prescaler = IWDG_PRESCALER_64;
    watchdog.Init.Reload = 499U; /* Nominal 1s at 32kHz LSI; not a stop-time guarantee. */
    __HAL_DBGMCU_FREEZE_IWDG(); /* Debug halt is safe because this target never drives. */
    if (HAL_IWDG_Init(&watchdog) != HAL_OK) {
        fatal(5U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *handle) {
    if (handle->Instance == USART2) {
        if (!uart_rx_queue_push(&rx_queue, rx_byte, HAL_GetTick())) {
            ++commissioning_rx_overflows;
            rx_restart_needed = true;
        }
        if (HAL_UART_Receive_IT(&uart2, &rx_byte, 1U) != HAL_OK) {
            ++commissioning_uart_errors;
            rx_restart_needed = true;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *handle) {
    if (handle->Instance == USART2) {
        ++commissioning_uart_errors;
        rx_restart_needed = true;
    }
}

static void recover_rx(void) {
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    if (HAL_UART_AbortReceive(&uart2) != HAL_OK) {
        fatal(6U);
    }
    __HAL_UART_CLEAR_OREFLAG(&uart2);
    uart_rx_queue_reset(&rx_queue);
    commissioning_application_stream_reset(&application);
    rx_restart_needed = false;
    if (HAL_UART_Receive_IT(&uart2, &rx_byte, 1U) != HAL_OK) {
        fatal(7U);
    }
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void) {
    HAL_UART_IRQHandler(&uart2);
}

void SysTick_Handler(void) {
    HAL_IncTick();
}

void NMI_Handler(void) { fatal(10U); }
void HardFault_Handler(void) { fatal(11U); }
void MemManage_Handler(void) { fatal(12U); }
void BusFault_Handler(void) { fatal(13U); }
void UsageFault_Handler(void) { fatal(14U); }

int main(void) {
    disable_motor_outputs();
    if (HAL_Init() != HAL_OK) {
        fatal(1U);
    }
    configure_clock();
    uart_rx_queue_init(&rx_queue);
    commissioning_application_init(&application);
    configure_uart();
    configure_watchdog();
    uint32_t last_telemetry_ms = HAL_GetTick() - TELEMETRY_PERIOD_MS;

    for (;;) {
        disable_motor_outputs();
        if (rx_restart_needed) {
            recover_rx();
        }
        uart_rx_item_t item;
        for (unsigned drained = 0U; drained < RX_DRAIN_BUDGET; ++drained) {
            if (rx_restart_needed || !uart_rx_queue_pop(&rx_queue, &item)) {
                break;
            }
            if ((uint32_t)(HAL_GetTick() - item.received_at_ms) > MAX_RX_AGE_MS) {
                ++commissioning_stale_rx_bytes;
                commissioning_application_stream_reset(&application);
                continue;
            }
            commissioning_application_receive(&application, &item.byte, 1U, item.received_at_ms);
        }
        const uint32_t now_ms = HAL_GetTick();
        if (uart2.gState != HAL_UART_STATE_READY
            && (uint32_t)(now_ms - last_telemetry_ms) > 100U) {
            ++commissioning_tx_errors;
            fatal(16U);
        }
        if ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS
            && uart2.gState == HAL_UART_STATE_READY) {
            if (!commissioning_application_step(&application, now_ms, tx_frame, sizeof(tx_frame))) {
                fatal(8U);
            }
            if (HAL_UART_Transmit_IT(&uart2, tx_frame, sizeof(tx_frame)) != HAL_OK) {
                ++commissioning_tx_errors;
                fatal(9U);
            }
            last_telemetry_ms = now_ms;
        }
        if (HAL_IWDG_Refresh(&watchdog) != HAL_OK) {
            fatal(15U);
        }
        __WFI(); /* Wake on UART or the 1ms SysTick, no blocking printf/HAL_Delay. */
    }
}
