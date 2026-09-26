# STM32F407VE board integration proposal

The supplied board schematic exposes the STM32F407VET6 GPIO on 2.54 mm headers, but the repository does not contain a CubeMX project or generated HAL startup code. The mapping below is a proposed pin allocation for that board, not a claim that it has been wired or electrically validated. The MCU alternate functions are cross-checked against the [ST STM32F407VE datasheet](https://www.st.com.cn/resource/en/datasheet/stm32f407ve.pdf); check the physical header silkscreen before wiring.

| Function | Candidate pin / peripheral | Notes |
|---|---|---|
| ESP32 motion bridge | USART2: PA2 TX, PA3 RX | Cross RX/TX to ESP32 GPIO18/17; 115200 baud initially; common signal ground. |
| Left wheel encoder A/B | TIM2_CH1/CH2: PA0/PA1 | Encoder mode; set timer modulus and edge-count mode from the selected motor/firmware configuration. |
| Right wheel encoder A/B | TIM4_CH1/CH2: PB6/PB7 | Encoder mode; do not also assign I2C1 to PB6/PB7. |
| TB6612 PWMA/PWMB | TIM3_CH1/CH2: PA6/PA7 | Candidate high-frequency PWM outputs. These pins are also wired to the board's touch-screen SPI connector; do not use that display/touch interface at the same time. |
| TB6612 AIN1/AIN2, BIN1/BIN2 | PB12/PB13/PB14/PB15 | GPIO direction outputs; map A/B polarity only after testing each motor with wheels lifted. |
| TB6612 STBY | PD2 | Add a hardware pull-down so the driver remains disabled while STM32 resets or boots. |
| MG90S servo PWM | TIM1_CH1: PA8 | Use a timer separate from the motor-PWM timer; pulse endpoints must be measured without a load. Power servo from a suitable 5–6 V rail, never 12 V or an MCU GPIO. |
| BMI088 I2C | I2C1: PB8 SCL, PB9 SDA | The board also routes PB8/PB9 to touch-screen signals; leave the screen/touch cable disconnected. Confirm the sensor board's pull-ups and 3.3 V logic. |
| TinyF range module 1 | USART3: PB10 TX, PB11 RX | UART frame format/baud still needs the exact module configuration. |
| TinyF range module 2 | UART4: PC10 TX, PC11 RX | Confirm both module TX outputs are 3.3 V logic before direct connection; VDD=5 V does not prove UART TX voltage. |
| Battery monitor (optional) | ADC1_IN14: PC4 | Battery must go through a resistor divider sized for the pack's maximum charged voltage; never connect the pack directly to an ADC pin. Divider values/calibration are not known yet. |

## TB6612 module connector labels

The supplied TB6612 schematic shows two 1×8 headers. Use the pin-1 marking on the physical board to confirm orientation before connecting:

| H1 pin | Signal | H1 pin | Signal |
|---:|---|---:|---|
| 1 | GND | 5 | STBY |
| 2 | PWMB | 6 | AIN2 |
| 3 | BIN2 | 7 | AIN1 |
| 4 | BIN1 | 8 | PWMA |

| H2 pin | Signal | H2 pin | Signal |
|---:|---|---:|---|
| 1 | GND | 5 | AO1 |
| 2 | BO1 | 6 | GND |
| 3 | BO2 | 7 | VCC (logic supply) |
| 4 | AO2 | 8 | VM (motor supply) |

For a nominal 12 V pack, VM is the motor supply and VCC is the 3.3 V logic supply; connect grounds as required by the driver design, and do not feed 12 V into STM32/ESP32/OpenMV GPIO or the servo. The STM32 board schematic itself recommends isolation for motor/high-interference loads. The TB6612 breakout has no galvanic isolation, so keep motor-current return paths away from logic/sensor grounds and add the protection/isolation required by the board maker before vehicle-power tests.

## Driver-current warning

The supplied MG513X image lists 0.36 A rated current and 3.2 A stall current. Toshiba's [TB6612FNG datasheet](https://toshiba.semiconductor.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG) lists 1.2 A average and 3.2 A peak output current. Thus a single motor's stated stall current already equals the IC's peak rating, while its continuous rating is lower; two jammed motors are not a safe operating case. A software duty cap is not a substitute for current limiting because PWM pulses can still reach stall current. Before powered drive tests, measure real motor current on a current-limited supply and strongly consider a driver with suitable continuous current and hardware current protection.

The MCU application core in [`include/rover_application.h`](include/rover_application.h) and [`src/rover_application.c`](src/rover_application.c) consumes sampled values and emits signed PWM requests, actuator commands, and RST1 telemetry. It does not configure the listed timers/UART/I2C/ADC or drive pins; that remains the board-specific HAL adapter. Never test with the wheels touching the floor until the motor-driver rating, stop behavior, and wheel direction have been verified.
