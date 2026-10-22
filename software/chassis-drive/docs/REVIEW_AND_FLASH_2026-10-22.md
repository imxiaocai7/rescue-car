# STM32 L298N 固件复查与烧录记录

日期：2026-10-22。状态：候选固件已生成并通过软件检查，尚未写入目标板。

## 本次修改

- PC6/PC7 在 HAL 初始化后立即作为低电平输出，先于 PLL/RNG 初始化；方向脚 PB11–PB14 同时拉低。定时器先复位，PWM 零占空比配置完成后才切换使能脚复用。复位期间仍依赖外部 ENA/ENB 下拉，软件不能替代该硬件措施。
- 移除原 TB6612 的 PB0/STBY 初始化及运行时写入。L298N 无 STBY，PB0 与核心板 SPI Flash CS 共网，应留给板载器件。
- 在处理完整运动请求的关中断区内复查 RX 故障。如果错误发生在出队后，不执行该请求，立即禁用输出，再由主循环清理串口并要求新会话。
- PC6→ENA、PC7→ENB，PB11/12/13/14→IN1/2/3/4。协议和 ESP32 固件不变。编码器采集和闭环仍未实现。

## 验证

- `python -m unittest discover -s tests -v`：31 项全部通过，覆盖实际 C 控制核心、协议、网关及 PC 客户端故障处理。
- PlatformIO `stm32-motion` 和 `stm32-uart-check` 两种构建通过；运动版 RAM 2356 bytes，Flash size report 9652 bytes。
- 两个环境均通过 `tools/check_stm32_artifact.py` 的向量、内存范围、处理函数和 HEX 检查。
- 运动版 BIN 10060 bytes；SHA256 `4361b9c1f4555aded4e564a62e14e10dc8aedf4b8eced55999eae2b92693be97`。
- 已复查差异与 `git diff --check`。新增启动时序与 RX 临界区代码经静态复核及目标编译，尚未完成板上验证；上述主机测试不验证物理电平。

## 硬件读取结果

用户确认 12V 已断开、STM32 仅电脑 USB 供电；随后退出 Keil 调试并将两板及 ST-Link 完整断电后接回。

- ST-Link 可识别 Cortex-M4；DBGMCU_IDCODE=`0x10076413`。
- 断电前后 FLASH_OPTCR 稳定为 `0x0FFFFFED`，RDP 字节为 `0xFF`，按 RM0090 属于 Level 1。
- 当前 UID、Flash 容量及 Flash 内容读取失败；因此不能取得当前程序完整备份。工具在容量读取失败后的 1024 KiB 回退值不能视为真实容量，工程仍为 F407VE 的 512 KiB。
- 旧记录（2026-10-11）OPTCR 为 `0x0FFFAAED`。本次不推断保护变化原因，不据此宣称芯片损坏。
- 原始日志保存在仓库 `logs/2026-10-22-flash/`，包含 `review_probe.txt` 和 `powercycle_probe.txt`。本轮没有擦除、写入 Flash 或修改选项字节，没有发送电机运动指令。

## 下一步

需要用户单独确认是否解除 Level 1 读保护：该操作会擦除整个主 Flash 和备份 SRAM，当前内容无法先读取保存。已有旧备份不代表当前板上程序。收到同意后，重新确认保护级别，只处理 Level 1→0，保留其他选项配置；成功后核对容量/身份，写入此次 HEX、读回校验，并在动力断开条件下验证默认禁用。若仍失败则保留日志继续排查，不反复盲目擦写。

依据：[ST RM0090，3.7.3 Read protection](https://community.st.com/ysqtg83639/attachments/ysqtg83639/stm32-mcu-products-forum/20920/1/STM32_Reference%20manual.pdf)。Level 1 下调试连接不能读取、擦除或编程主 Flash；降到 Level 0 会触发主 Flash 与备份 SRAM 的整体擦除。
