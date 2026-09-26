# ESP32-S3 相机与运动双向网关

本模块负责 OpenMV 图像帧上行，以及电脑与 STM32 之间的原始双向 UART/TCP 转发。它不解析运动控制帧，也不代替 STM32 的命令校验、租约看门狗或电机控制。

## 硬件连接

| OpenMV H7 Plus | ESP32-S3-N16R8 | 说明 |
|---|---|---|
| P4 / UART3 TX | GPIO16 / UART1 RX | OpenMV → ESP32 |
| P5 / UART3 RX | GPIO15 / UART1 TX | ESP32 → OpenMV 的 `R` 请求 |
| GND | GND | 必须共地 |

运动串口当前使用 ESP32-S3 GPIO18 RX、GPIO17 TX、115200 bit/s；这是按开发板常见空闲 GPIO 预留的候选配置，接线前需核对你手中开发板的引脚图和占用情况。若 STM32 使用 USART2 默认映射，可先按下面的交叉连接设计，实际 PA2/PA3 是否可用还要核对底板：

| ESP32-S3 | STM32F407 | 说明 |
|---|---|---|
| GPIO17 TX | USART2_RX（常见映射 PA3） | `RCM1` 命令下行 |
| GPIO18 RX | USART2_TX（常见映射 PA2） | `RST1` 遥测上行 |
| GND | GND | 必须共地；不要互接 5 V 电源 |

两边 UART 都按 3.3 V 逻辑设计。若改用其他 STM32 UART 引脚，只需改 STM32 外设配置；若 ESP32 GPIO 被底板占用，再同步更改 `src/main.cpp` 中 `kMotionRxPin` / `kMotionTxPin`。

两块板分别供电，UART 电平按 3.3 V 逻辑连接。不要把 5 V 接入 UART GPIO。初次联调可将 `src/main.cpp` 和 `software/openmv/camera_uart.py` 的波特率同时改为 115200；验证稳定后再同时切到 921600。

## Wi-Fi 与电脑接收

ESP32 启动后创建 2.4 GHz Wi-Fi 热点：

- SSID：`RescueBot-S3`
- 密码：`rescue123`
- ESP32 地址：`192.168.4.1`
- TCP 图像端口：`5000`
- TCP 运动/遥测端口：`5001`

先让 Windows 笔记本连接该热点，再在仓库根目录运行：

```powershell
python software/pc/receive_camera.py 192.168.4.1 --port 5000
```

每次电脑连入后，ESP32 才向 OpenMV 请求一帧。ESP32 在 PSRAM 暂存完整 JPEG、核对 IEEE CRC32 和 JPEG 首尾标志后，才把原始 26 字节头及图像发给电脑。电脑完成解码/推理和显示后回传 8 字节 `ACK1 + seq(u32 little-endian)`；只有帧号匹配才请求 OpenMV 下一帧，超过 15 秒未确认会断开该客户端，避免视觉端落后时积压旧图。

电脑另连 `192.168.4.1:5001`。此 TCP 连接与图像 ACK 通道独立：电脑发送 20 字节 `RCM1` 运动帧，ESP32 原样转给 STM32；STM32 产生 38 字节 `RST1` 遥测帧后，ESP32 原样回传电脑。帧格式见仓库根目录的 [`../MOTION_PROTOCOL.md`](../MOTION_PROTOCOL.md)。客户端断开时 ESP32 不伪造停止帧，STM32 必须靠命令租约到期归零；因此电脑应周期发送带租约的新命令。ESP32 网桥只做字节搬运，不确认某个命令已被 STM32 接受，电脑可从遥测中的 `last_command_sequence` 确认。

## 构建与烧录

需要 PlatformIO Core；在本目录执行：

```powershell
pio run
pio run --target upload --upload-port COMx
pio device monitor --port COMx --baud 115200
```

`platformio.ini` 当前按照片中 ESP32-S3-N16R8（16 MB QIO Flash、8 MB OPI PSRAM）配置。第三方开发板版本可能不同；首次烧录前请核对模组丝印及 Flash/PSRAM 类型。若无法识别 PSRAM，程序会停止并打印错误，不会退化为不校验的转发。

电脑可先启动 TCP 接收程序，再复位 ESP32。OpenMV 脚本须已运行，且与相机 UART 波特率一致。运动网桥可由 [`pc/motion_link.py`](../pc/motion_link.py) 的 `MotionLinkClient` 使用；该连接须由集成控制循环维护，当前尚未接入 `view_camera.py`。串口监视器会显示两个服务地址、连接状态及相机帧日志。
