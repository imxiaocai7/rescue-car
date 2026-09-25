# ESP32-S3 相机 Wi-Fi 网关

本模块只负责 OpenMV 图像帧的 UART 接收、完整性校验和 TCP 转发；不接 STM32、不发电机或舵机命令。

## 硬件连接

| OpenMV H7 Plus | ESP32-S3-N16R8 | 说明 |
|---|---|---|
| P4 / UART3 TX | GPIO16 / UART1 RX | OpenMV → ESP32 |
| P5 / UART3 RX | GPIO15 / UART1 TX | ESP32 → OpenMV 的 `R` 请求 |
| GND | GND | 必须共地 |

两块板分别供电，UART 电平按 3.3 V 逻辑连接。不要把 5 V 接入 UART GPIO。初次联调可将 `src/main.cpp` 和 `software/openmv/camera_uart.py` 的波特率同时改为 115200；验证稳定后再同时切到 921600。

## Wi-Fi 与电脑接收

ESP32 启动后创建 2.4 GHz Wi-Fi 热点：

- SSID：`RescueBot-S3`
- 密码：`rescue123`
- ESP32 地址：`192.168.4.1`
- TCP 端口：`5000`

先让 Windows 笔记本连接该热点，再在仓库根目录运行：

```powershell
python software/pc/receive_camera.py 192.168.4.1 --port 5000
```

每次电脑连入后，ESP32 才向 OpenMV 请求一帧。ESP32 在 PSRAM 暂存完整 JPEG、核对 IEEE CRC32 和 JPEG 首尾标志后，才把原始 26 字节头及图像发给电脑。电脑完成解码/推理和显示后回传 8 字节 `ACK1 + seq(u32 little-endian)`；只有帧号匹配才请求 OpenMV 下一帧，超过 15 秒未确认会断开该客户端，避免视觉端落后时积压旧图。

## 构建与烧录

需要 PlatformIO Core；在本目录执行：

```powershell
pio run
pio run --target upload --upload-port COMx
pio device monitor --port COMx --baud 115200
```

`platformio.ini` 当前按照片中 ESP32-S3-N16R8（16 MB QIO Flash、8 MB OPI PSRAM）配置。第三方开发板版本可能不同；首次烧录前请核对模组丝印及 Flash/PSRAM 类型。若无法识别 PSRAM，程序会停止并打印错误，不会退化为不校验的转发。

电脑可先启动 TCP 接收程序，再复位 ESP32。OpenMV 脚本须已运行，且与网关波特率一致。串口监视器会显示热点地址、客户端连接、图像序号、尺寸及校验错误。
