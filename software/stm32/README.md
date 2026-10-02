# STM32 第二阶段：通信验收固件

状态：**软件工程已构建，尚未烧录或做实物验收**。用户当前没有硬件在身边，本阶段不执行任何上传、
ST-Link 连接、擦除或复位操作，也不修改 ESP32/OpenMV 固件和网络配置。

## 本版能做什么

- 目标是 STM32F407VET6：512KiB Flash，128KiB 普通 SRAM；64KiB CCM 暂不使用。
- 使用 ST CMSIS 启动文件、链接脚本和 STM32Cube F4 HAL；通过 PlatformIO 构建，不要求先生成 CubeMX 工程。
- 使用内部 HSI 16MHz，暂不猜测板上晶振频率；SysTick 1ms。
- USART2：PA2 TX、PA3 RX，115200 / 8N1，以中断方式双向收发。
- 接收 `RCM1`，**只接受左右轮均为零、舵机 HOLD 的帧**。非零轮速和舵机动作拒收，不更新确认序号。
- 20Hz 回传 `RST1`，包含启动时间、上次接受序号。CRC 错误、重复/旧序号不会刷新租约。
- 接收队列为单中断生产者 / 单主循环消费者，保存字节接收时间；溢出或 UART 错误重置部分帧与租约，保留防重放序号。
- 主循环丢弃在 MCU 接收队列里超过 100ms 的字节。这里不是电脑发送时间，不能保证清除 ESP32/TCP 内的旧帧。
- 配置独立看门狗，LSI 标称 32kHz 时约 1s；实际时间必须实测。调试器暂停时冻结看门狗。

## 永久禁止运动

`f407-communication-only` 是独立目标，不存在“修改一个运行参数就启用电机”的入口。
上电进入 main 后，立即把 PD2/STBY、PA6/PA7 和 PB12/PB13/PB14/PB15 配置为低电平输出；
主循环持续保持，硬故障处理也重新拉低。没有初始化电机 PWM、编码器或舵机定时器。

电路复位到程序执行前，引脚还可能处于高阻状态。因此 **STBY 的外部下拉、正确接线和动力电隔离仍然必要**。
软件通过编译不证明引脚电压、板上外设冲突、看门狗复位或电机禁用已经在实物上验证。
PA13/SWDIO、PA14/SWCLK 不被本程序重新配置；触摸屏/显示扩展先不要接入共用引脚。

没有编码器和其他传感器采样时，遥测不会伪造有效状态。RST1 `EMERGENCY_STOP` 位常置 1，表示本版
**软件通信验收锁定**，不是宣称已经连接或测过物理急停按钮。`LAST_COMMAND_VALID` 表示曾接受该序号，
即使租约已经到期也保留，不表示当前允许驱动。其他有效位为 0，编码器累计值为 0，测距为 0xFFFF。

第一阶段的手动控制界面看到这些状态会锁定，这是预期行为，不要通过伪造编码器有效或清除急停位绕过。
阶段三才接入真实编码器和电机适配层。

只发零速的联调工具已补齐，见 [电脑通信验收说明](../pc/MOTION_COMMISSIONING.md)。没有硬件时可运行
`python software/pc/probe_motion.py --demo`，仅测试本机模拟链路。

## 仅构建，不烧录

在 `C:\Users\imxiaocai\Desktop\gc` 根目录运行：

```powershell
& .\.venv-vision\Scripts\python.exe -m platformio run --project-dir software/stm32
& .\.venv-vision\Scripts\python.exe software/stm32/tools/check_firmware.py
```

固定依赖为 PlatformIO 平台 ststm32 20.0.0、Cube F4 1.28.3、链接脚本包 0.3.0 和 ARM GCC 包 1.70201.0
（GCC 7.2.1）。本机 `.venv-vision` 已有 PlatformIO Core 6.1.18，构建成功；若需在新电脑准备环境，
先安装支持 Tk/PlatformIO 的 Python，并安装 `platformio==6.1.18`。首次构建会下载官方工具包。

生成文件位于 `software/stm32/.pio/build/f407-communication-only/`：

- `firmware.elf`：包含调试符号、启动向量与链接结果。
- `firmware.bin`：原始烧录数据；以后经确认后，起始地址应为 **0x08000000**。
- `firmware.hex`：包含地址的 Intel HEX。

这些是可用于后续烧录验收的候选文件，不是实车验收合格固件。`.pio/` 已被 Git 忽略，只提交源代码和配置。
`check_firmware.py` 只读文件，检查 ARM 类型、Flash/SRAM 范围、启动栈、中断向量和未链接运动控制符号，
输出中明确标记 `hardware_verified: false`。

## 本机 C 与协议兼容测试

```powershell
& .\software\stm32\tools\test_host.ps1
& .\.venv-vision\Scripts\python.exe -m unittest discover -s software/pc -p "test_*.py"
```

脚本使用本机 GCC，严格警告编译 7 个 C 测试程序，包括原有轮速/编码器核心，以及本版通信验收和接收队列。
临时测试程序写入独立临时目录，不删除已有文件或提交。
`test_stm32_commissioning.py` 还把 Python 生成的真实 RCM1 字节送进本机编译的 C 核心，
再用 Python 解码 C 生成的 RST1，验证协议兼容、CRC 恢复、序号回绕以及非零轮速/舵机拒收。
它不运行 MCU 寄存器、UART 中断或物理引脚。

## 以后硬件在身边时的验收顺序（本轮不执行）

1. 电机动力电池与 TB6612 VM 断开，STM32/ESP32 暂用 USB 供电，确认 BOOT0 为 0。
2. 核对 ST-Link 实物型号与排针：SWDIO→PA13、SWCLK→PA14、GND→GND；NRST 可选，但须核对排针。
   某些 ST-Link 的 3.3V 是电源输出，另一些是目标参考电压输入，不能混用。USB 供电时不要盲接第二路电源。
3. 只读识别芯片、Flash 大小和目标电压；核对 MCU 丝印。先备份已有 Flash；若读保护阻止备份，停止询问，
   不自动解除保护或全片擦除。烧录会覆盖相关 Flash，确认备份与授权后才能继续。
4. 核对 ESP32 GPIO17 TX→STM32 PA3 RX，STM32 PA2 TX→GPIO18 RX，并共地；OpenMV 原接线不用拆。
5. 经授权再烧录、校验、复位。检查 STBY/PWM/方向引脚始终低电平。
6. 运行只发零速的通信验收工具，检查持续状态回传、确认序号、重连和复位后的序号同步。
7. 记录 UART 差错、长期稳定性、看门狗行为和引脚电压；通过后才能进入真实编码器接入。

电池满电电压、驱动容量和限流未确认前仍不接电机动力电。TB6612 不能把 3.2A/10ms 单次峰值当作持续能力。

## 软件验证记录（2026-10-03）

- ARM 交叉编译成功，生成 ELF/BIN/HEX；构建报告 Flash 7464 字节、静态 RAM 2328 字节。
- BIN 实际文件长度 7872 字节（包含向量与对齐区域）；结构检查通过，初始栈为 0x20020000。
- 7 个本机 C 测试程序通过；固件模块提交前全量 149 项测试通过，包含 7 项新增 Python↔C 检查。
- 硬件启动、SWD、UART、电平和看门狗的实物验收均未执行。
