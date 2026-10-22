# ESP32-S3-N16R8 通信桥接固件

> 2026-10-22：当前已重写为 v3，同时支持运动端口 6002 和诊断端口 6001。当前构建、通信约定、控制窗口及验收状态见 [电脑控制说明](PC_CAR_CONTROL_2026-10-22.md)；下文仅诊断描述及旧时间参数属于历史记录。

只为本项目诊断协议服务，不依赖旧固件、不处理图像、不接摄像头。
目标16MB QIO Flash、8MB OPI PSRAM。程序不依赖PSRAM存储，启动报告检测到的容量。
若Flash不是16MB，则不开放服务。实际模块丝印和检测仍须核对。

## 构建，不上传

```powershell
& .\.venv\Scripts\python.exe -m platformio run -d firmware/esp32
```

产物位于 `firmware/esp32/.pio/build/esp32-s3-bridge/`。
`firmware.bin`仅是应用，不能单独从地址0烧录；应让PlatformIO按配置上传引导、分区和应用。

## USB与未来上传

ESP32单独用USB供电，USB-UART或原生USB口都可用，串口调试115200。
已启用原生USB CDC（硬件Serial/JTAG模式），日志同时输出UART0和原生USB，不等待电脑打开监视器。
每5秒报告固件标识、Flash/PSRAM容量、热点/IP/待确认状态，避免错过上电日志后以为程序没有运行。
若接口标签不清晰，先发实物照片，不盲选COM号、不盲安装驱动。
先核对ROM芯片信息和16MB Flash、备份原Flash，再上传。上传会覆盖旧ESP32程序，本项目不保存图传功能。
原程序备份保存到Git忽略的backups目录；如果有安全启动/Flash加密等限制，停止询问。

未来确认COM号、备份后才执行（把COMx替换为实际串口）：

```powershell
& .\.venv\Scripts\python.exe -m platformio run -d firmware/esp32 -t upload --upload-port COMx
& .\.venv\Scripts\python.exe -m platformio device monitor -p COMx -b 115200
```

自动下载失败时，按住BOOT、按一下RESET/EN、松开BOOT，再重试。只操作ESP32，别操作另一块板子。

## Wi-Fi诊断模式

- 临时热点：ChassisDrive-Setup；密码：Chassis2026。
- 默认IP：192.168.4.1，以串口实际报告为准。
- TCP端口：6001。只允许一个热点客户端和一个TCP会话。
- UART2：GPIO17 TX、GPIO18 RX，115200 8N1。
- 网关核对CRC/操作类型，只转发PING/STOP；等待STM32回复，绝不自行伪造回复。
- 每次只允许一个待确认请求，400ms没有有效回复关闭客户端。
- 3秒空闲关闭；残帧500ms过期。断线后清空解析和待确认状态。
- 固件标识、状态、序号错误时关闭，不自动解锁或重放。

热点只是首次通信验收方案，不代表比赛网络架构定案。此热点不提供互联网。
不要自动切换电脑网络；需要保留网络时先安排另一条联网通道或另行配置STA模式。
默认密码仅用于台架，部署时修改。CRC和固件标识不是身份认证，不连接不可信网络。

未来连接热点并完成两块板子的烧录后，在项目根目录运行：

```powershell
python pc/probe.py --host 192.168.4.1 --port 6001 --allow-hardware --count 30
```

报告中的确认来自STM32；`hardware_verified: false`仍表示没有验收电机或整车安全。

## 软件验证记录（2026-10-07）

启用双USB日志与周期容量报告后的ESP32-S3交叉编译通过，Flash报告723261字节，静态RAM45112字节。
应用BIN长度723632字节，esptool只读镜像检查的校验和与验证Hash均有效。
编译/镜像检查不证明USB、Flash实测容量、Wi-Fi、UART或复位行为。

25项软件测试通过。COM8最初返回WinError31；用户按BOOT重新上电后端口恢复，
再通过esptool的usb_reset成功连接。目标确认为ESP32-S3 v0.2，16MB Flash、8MB内嵌PSRAM、40MHz晶振。
安全启动及Flash加密关闭。完整16MiB备份及板上摘要校验完成，新程序写入校验及启动日志检查通过。
详细记录见 [ESP32_BRINGUP_2026-10-07.md](ESP32_BRINGUP_2026-10-07.md)；STM32和端到端链路尚未验收。
