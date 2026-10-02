# STM32F407VE board integration proposal

The supplied board schematic exposes the STM32F407VET6 GPIO on 2.54 mm headers. A buildable, communication-only PlatformIO/Cube HAL target now exists; see [README.md](README.md). It permanently disables motor outputs and does not sample encoders or drive motors. The mapping below remains a pin allocation, not a claim of wiring/electrical validation. The MCU alternate functions are cross-checked against the [ST STM32F407VE datasheet](https://www.st.com.cn/resource/en/datasheet/stm32f407ve.pdf); check the physical header silkscreen before wiring.

| Function | Candidate pin / peripheral | Notes |
|---|---|---|
| ESP32 motion bridge | USART2: PA2 TX, PA3 RX | Cross RX/TX to ESP32 GPIO18/17; 115200 baud initially; common signal ground. |
| Left wheel encoder A/B | TIM2_CH1/CH2: PA0/PA1 | Encoder mode x4; configure 1560 counts/output-shaft revolution, then verify by turning the wheel exactly once. |
| Right wheel encoder A/B | TIM4_CH1/CH2: PB6/PB7 | Encoder mode x4; configure 1560 counts/output-shaft revolution; do not also assign I2C1 to PB6/PB7. |
| TB6612 PWMA/PWMB | TIM3_CH1/CH2: PA6/PA7 | Candidate high-frequency PWM outputs. These pins are also wired to the board's touch-screen SPI connector; do not use that display/touch interface at the same time. |
| TB6612 AIN1/AIN2, BIN1/BIN2 | PB12/PB13/PB14/PB15 | GPIO direction outputs; map A/B polarity only after testing each motor with wheels lifted. |
| TB6612 STBY | PD2 | Add a hardware pull-down so the driver remains disabled while STM32 resets or boots. |
| MG90S servo PWM | TIM1_CH1: PA8 | Use a timer separate from the motor-PWM timer; pulse endpoints must be measured without a load. Power servo from a suitable 5–6 V rail, never 12 V or an MCU GPIO. |
| BMI088 I2C | I2C1: PB8 SCL, PB9 SDA | The board also routes PB8/PB9 to touch-screen signals; leave the screen/touch cable disconnected. Confirm the sensor board's pull-ups and 3.3 V logic. |
| TinyF range module 1 | USART3: PB10 TX, PB11 RX | MINIF/DVP0501C1; 115200 baud, 8N1, 30 Hz; PB10 to module pin 2 RXD, PB11 from pin 3 TXD. |
| TinyF range module 2 | UART4: PC10 TX, PC11 RX | Same profile; PC10 to module pin 2 RXD, PC11 from pin 3 TXD. Each module needs its own UART. |
| Battery monitor (optional) | ADC1_IN14: PC4 | Battery must go through a resistor divider sized for the pack's maximum charged voltage; never connect the pack directly to an ADC pin. Divider values/calibration are not known yet. |

## TB6612 module connector labels

Use the actual module's signal names, NOT the older schematic's H1 pin numbers: its AIN1/AIN2 ordering differs from the supplied module photo. With the photo upright, left to right:

| Row | Signals |
|---|---|
| Top | GND, PWMB, BIN2, BIN1, STBY, AIN1, AIN2, PWMA |
| Bottom | GND, BO1, BO2, AO2, AO1, GND, VCC, VM |

VM is the motor supply, while VCC should be the 3.3 V logic supply for STM32 logic compatibility. A nominal "12 V" label does not establish safe VM: verify the battery's full-charge voltage against the 2.5-13.5 V operating range. Connect grounds as required by the driver design; never feed motor voltage into MCU GPIO or the servo. The STM32 schematic recommends isolation for motor/high-interference loads. The breakout has no galvanic isolation: keep motor-current return paths away from logic/sensor grounds and add the required protection before vehicle-power tests.

## Confirmed motor and range-sensor profile

The motor is the Hall-encoder `MG513P30_12V`: 12 V, 1:30 reduction, 366 +/- 26 rpm no-load and 293 +/- 21 rpm rated output speed. Rated current is 0.36 A and stall current is 3.2 A per motor. Its AB encoder accepts 3.3-5 V and provides 13 pulses per motor-shaft revolution. With the 1:30 gearbox and STM32 timer encoder mode counting all four AB edges, configure `13 * 30 * 4 = 1560` counts per output-shaft revolution. This is now the firmware default in [`include/rover_hardware_profile.h`](include/rover_hardware_profile.h), but the sign and exact one-revolution count must still be checked on each assembled wheel.

The two range modules are `MINIF/DVP0501C1` TinyF units. Power each from 5 V and connect its 3.3 V LVTTL UART directly to the assigned STM32 UART with a common ground. The default is 115200 baud, 8N1 and 30 Hz. Each module actively outputs ASCII frames such as ` 327, 61\n`, meaning 327 mm with confidence 61; the documented range is 20-4500 mm and confidence is 0-62. There is no CRC, so the board adapter must validate the leading space, comma-space separator, newline terminator, digit lengths and numeric ranges. Do not join the two module TX outputs.

## Driver-current warning

The MG513P30_12V manual confirms 0.36 A rated current and 3.2 A stall current per motor. The supplied Toshiba TB6612FNG datasheet (2012-11-01) specifies 1.2 A average absolute maximum and 3.2 A only for a 10 ms SINGLE pulse (2 A repeated pulses: 20 ms, duty <=20%). These are not a continuous 3.2 A rating. Two jammed motors are not a safe operating case. A software duty cap is not current limiting because PWM pulses can still reach stall current. Before powered tests, measure actual current on a current-limited supply and consider a suitably rated, hardware-current-protected driver.

The MCU application core in [`include/rover_application.h`](include/rover_application.h) and [`src/rover_application.c`](src/rover_application.c) consumes sampled values and emits signed PWM requests, actuator commands, and RST1 telemetry. It does not configure the listed timers/UART/I2C/ADC or drive pins; that remains the board-specific HAL adapter. Never test with the wheels touching the floor until the motor-driver rating, stop behavior, and wheel direction have been verified.
