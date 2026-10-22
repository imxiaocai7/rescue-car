# 首次接线与烧录（只接STM32/ESP32）

此文保留首次诊断阶段的流程。2026-10-11 新运动阶段请优先看 [独立模块逐线接线表](MOTOR_WIRING_2026-10-11.md) 与 [运动构建/协议](MOTION_PROTOCOL.md)；新代码已实现、未刷写，旧诊断固件仍在板上。

本项目目录：`C:\Users\imxiaocai\Desktop\chassis-drive`。
2026-10-07：ESP32目标识别、完整Flash备份/校验、新固件烧录/校验和启动日志检查已完成。
使用原生USB COM8，详见 [ESP32实物记录](ESP32_BRINGUP_2026-10-07.md)。
STM32也已完成识别、完整备份/校验、烧录/校验及运行寄存器检查，见 [STM32实物记录](STM32_BRINGUP_2026-10-07.md)。
2026-10-10：两板实际UART收发、Wi-Fi端到端往返、600次持续通信及TCP/Wi-Fi重连已初步验收，见 [通信链路实物验收](COMMUNICATION_CHECK_2026-10-10.md)。电机控制仍未实现。
COM号在重插或换电脑后仍需重新检查，不假定始终COM8。

## 供电选择

STM32使用商品图右上方Mini-USB供电；ESP32使用自己的USB供电。
ST-Link只连接SWD信号与GND，3.3V和5V全部留空。两板5V/3.3V不互接。
若实物接口与商品图不同，请先核对；不把接口类型认错后硬插线。
驱动模块、电机、动力电池、摄像头与舵机全部不接。
更改接线前拔掉所有USB，再连接杜邦线。供电时保持信号线连接，两板同时上电。

## 六根杜邦线

| 来源 | 目的 |
|---|---|
| ESP32 GPIO17 (TX) | STM32 PA3 (RX) |
| ESP32 GPIO18 (RX) | STM32 PA2 (TX) |
| ESP32 GND | STM32 GND |
| ST-Link SWDIO | STM32 PA13，用户提供P1图的7号 |
| ST-Link SWCLK | STM32 PA14，用户提供P1图的9号 |
| ST-Link GND | STM32 GND，用户提供P1图的4号 |

蓝色Risym ST-Link外壳图：2号SWCLK，4号SWDIO，5/6号GND。
此编号与STM32的P1编号是两套不同编号，不能一一顺接。
用户提供的P1电路图中，1号接3V3；2号虽标Vsupply却画为GND。这两脚在本供电方案中都不接。
不接SWIM、TRST、TDI、TDO或RST。PA13/PA14只做下载，不接ESP32。
实物1号位置必须按三角/数字/焊盘标识确认；缺口与原理图方位不能直接确定针号。
已有原理图无需反复上传，仅在编号无法确认时需要实物近照。

## 三个USB连接

1. STM32 Mini-USB → 电脑USB：本版仅供电，不枚举USB串口。
2. ESP32 USB-UART或原生USB → 电脑USB：供电、下载和115200串口日志。
3. ST-Link → 电脑USB：下载STM32；不会作为COM串口出现。

电脑仅出现蓝牙COM4/COM5不说明板子已经连接。需检查ESP32真实USB串口与ST-Link设备。
USB-UART端口可能使用CP210x/CH34x等芯片，驱动需按硬件ID选取；不盲装未知驱动。

## 构建检查

在项目根目录：

```powershell
& .\.venv\Scripts\python.exe -m unittest discover -s tests -v
& .\.venv\Scripts\python.exe -m platformio run -d firmware/stm32
& .\.venv\Scripts\python.exe tools/check_stm32_artifact.py
& .\.venv\Scripts\python.exe -m platformio run -d firmware/esp32
```

以上命令不访问硬件、不上传。默认电脑演示为 `python pc/probe.py --demo`。

## STM32识别与烧录顺序

先连接并只读核对型号、供电、512KiB Flash和读保护。检查BOOT0处于Flash启动配置。
备份0x08000000开始的完整512KiB到backups目录，再核对新HEX后写入并校验。
已安装PlatformIO工具包OpenOCD 3.1200.7；初次SWD连接失败，用户调整后已完成STM32识别、备份和烧录。
后续重烧仍要核对实物与备份，不能因为本次成功就跳过这些步骤。
可以安装ST官方STM32CubeProgrammer及ST-Link驱动，连接模式选择SWD，初次频率可选400kHz。
连接失败不要直接执行全片擦除、解除读保护、修改Option Bytes或改电源线路。
本项目HEX：`firmware/stm32/.pio/build/stm32-uart-check/firmware.hex`。
BIN起始地址为0x08000000。上传后使用工具校验再复位启动；USB串口不用于STM32烧录。
需要帮助操作软件时，可逐步指导；不在未确认目标时直接执行上传命令。

## ESP32识别与烧录顺序

确认ESP32-S3-N16R8和真实COM号，停止其他串口监视器。
只读确认芯片、Flash容量及安全设置，备份原16MiB Flash到backups；如读出受限则停止询问。
本新项目将替换ESP32旧应用，不保留图传功能。
备份完成后才按 [ESP32说明](ESP32.md) 的PlatformIO上传命令烧录，以便一起写入引导和分区。
不把firmware.bin单独从0地址写入，不执行不必要的全片擦除。

## 验收与边界

监视器应输出CHASSIS_DIAGNOSTIC_BRIDGE、Flash容量、热点名和IP。
电脑切换热点可能失去互联网，先安排联网通道，不自动切换。
确认进入热点后用 `python pc/probe.py --host 192.168.4.1 --allow-hardware --count 30` 检查确认。
测试无回传：先停止诊断、两板断电，再临时断开ESP32 GPIO17到STM32 PA3的线。
两板重新上电后应无法确认，证明ESP32没有代答；再次全部断电恢复连线，再上电并手动重连测试。
USB拔插前先停止诊断，两块板子信号连接时尽量同时断电，避免未供电GPIO被反向供电。
测量各板电源和UART逻辑电平，并测试复位后恢复、不同次数重连和较长运行稳定性。

本阶段只证明诊断链路；不证明编码器、PWM、电机、物理急停或失联停车能力。

## 软件检查记录（2026-10-07）

25项测试通过，包括C协议与网关核心、Python/C兼容、本机TCP、超时、断线、CRC恢复、错误回复与复位检查。
两块板子的交叉编译成功。ESP32后来已完成Flash读取、备份、上传校验和复位后启动日志检查；
STM32后来也已完成完整备份、烧录校验、复位和运行寄存器检查；电脑网络没有切换。
确认往返时间包含系统调度、无线和UART处理，不是单向延迟；计时分辨率也可能让本机演示显示0ms。
