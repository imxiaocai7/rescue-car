# STM32 UART通信验收固件

目标：STM32F407VET6，512KiB Flash、128KiB普通SRAM。不使用CCM、板载SRAM、FLASH、屏幕或电机。
内部HSI 16MHz，不依赖尚未核对的外部晶振。只初始化USART2 PA2(TX)/PA3(RX)，115200 8N1。
PA13/PA14保留SWD；上电不配置电机GPIO，不启动PWM或编码器。任何驱动模块都必须保持断开。

固件只处理本项目CDP1 PING/STOP；回复CDS1。协议标志1表示“没有实现电机控制”，不是测过物理停机输出。
不生成伪编码器值，不存在非零运动授权入口。错误CRC或其他操作忽略，不回复。
接收中断进入256槽环形队列，主循环限量处理；溢出/错误丢弃队列和残帧后重启接收。
MCU队列内超过200ms的字节会丢弃，但这不是端到端命令年龄判断。
独立看门狗名义约1秒，暂停调试时冻结；时间、电平和恢复逻辑尚须实测。

## 构建（不烧录）

在新项目根目录执行：

```powershell
& .\.venv\Scripts\python.exe -m platformio run -d firmware/stm32
```

输出位于 `firmware/stm32/.pio/build/stm32-uart-check/`，包含firmware.elf/bin/hex。
BIN未来经确认后写入起始地址0x08000000；HEX自带地址。

编译后可运行只读结构检查：

```powershell
& .\.venv\Scripts\python.exe tools/check_stm32_artifact.py
```

2026-10-07：交叉编译通过，Flash报告6504字节、静态RAM2184字节，BIN长度6912字节。
ARM类型、普通SRAM范围、Flash范围、向量与中断安装检查通过；没有链接HAL定时器控制实现。
这些检查不替代SWD、上电、UART或看门狗实测。

## 未来烧录前检查

1. 实物确认MCU丝印为STM32F407VET6，供电正常，BOOT0选择Flash启动。
2. 确认SWDIO/PA13、SWCLK/PA14、GND及调试座的实际1号方向，不能凭原理图方位猜测。
3. STM32用自己的USB供电，ST-Link所有电源脚留空；不并联供电，不接动力电。
4. 先只读识别目标，备份已有512KiB Flash到Git忽略的backups目录；有读保护就停止，不自动解除。
5. 核对候选固件及备份后，经用户确认再写入、校验与复位。只读识别失败时不要盲擦除。

ST-Link的RST标签可能是STM8接口，不自动当作STM32 NRST使用。初次SWD联调不接它。
若后续使用STM32CubeProgrammer，可载入HEX并启用写后校验；不得在未核对目标时执行全片擦除。

## 最初实物连接检查（2026-10-07，后来已恢复并烧录）

- Windows已识别STM32 STLink，VID0483/PID3748。
- 已安装官方工具包中的OpenOCD 3.1200.7；识别下载器为STLINK V2J46S7。
- 下载器报告参考电压3.273018V，此数值不能代替目标板实际供电检查。
- 直接SWD 400kHz、100kHz及HLA兼容100kHz均在init阶段报告unable to connect to the target。
- 因此未读到MCU ID、Flash容量或读保护；未备份、擦除、写入或复位STM32。
- 候选固件结构检查仍通过，BIN SHA256为6c48f7fb28cf5b04f67e4e9931cfc8c606a4f14b991a5832c8b98f50aa6457dc。

下一步由用户检查STM32电源灯、USB供电，以及SWDIO→PA13、SWCLK→PA14、GND共地。
调试座编号有疑问时可改用明确标注的PA13/PA14排针。改线必须先断电。
若供电与线路已确认，再考虑旧程序关闭SWD时的连接恢复，不盲擦除或解除读保护。

用户调整连接后，已读到Cortex-M4、512KiB Flash和未保护状态，完成完整Flash备份/校验、新程序写入/校验与复位。
运行中的SysTick递增、USART2/PA2/PA3寄存器配置符合设计；实际UART收发及整条Wi-Fi链路尚未验收。
最终结果见 [STM32_BRINGUP_2026-10-07.md](STM32_BRINGUP_2026-10-07.md)，上方失败记录仅为历史检查。
