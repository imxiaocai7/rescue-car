# 通信链路实物验收（2026-10-10）

结果：电脑 Wi-Fi → ESP32-S3 → UART → STM32F407 → ESP32 → 电脑的真实诊断往返已经初步验收。首轮 30 次、连续 600 次、两次 TCP 重连各 30 次以及 Wi-Fi 断开再连接后的 30 次全部通过，另有掉线前 1 次确认，共 721 次真实回复。没有重新烧录两板。

## 重新上电后的当前状态

用户让两板断电并正常重新上电后，扫描发现 ChassisDrive-Setup，BSSID 为 ba:1f:3f:c4:6e:dc，2.4 GHz 信道 6，与既有 ESP32 的地址一致。添加当前用户的手动连接配置后，电脑接入此热点，WLAN 获得 192.168.4.2，目标 192.168.4.1:6001 经 WLAN 直连。

手机 USB 共享持续工作：Apple Mobile Device Ethernet（以太网 3）为 172.20.10.2，网关 172.20.10.1。连接 ESP32 热点后，以及 Wi-Fi 断开/重连测试后，显式绑定手机地址访问百度 HTTPS 都返回 HTTP 200；最后的互联网目标路由也选择手机接口。电脑当前 WLAN 连接 ESP32，互联网使用手机共享。

刚重新上电时 COM8 尚未出现在枚举中，后续重新枚举已恢复 COM8，VID 303A/PID 1001，序列号 B8:1F:3F:C4:6E:DC。最后读取约 12 秒，收到 3 条正常周期状态：

```text
CHASSIS_DIAGNOSTIC_BRIDGE v1 Flash=16777216 PSRAM=8385847 SSID=ChassisDrive-Setup IP=192.168.4.1 TCP=6001 pending=0
```

## 真实诊断测试结果

| 测试 | 确认数 | 最大往返耗时 | 结果 |
|---|---:|---:|---|
| 首轮真实目标测试 | 30/30 | 16 ms | 通过 |
| 单个 TCP 会话连续通信 | 600/600 | 16 ms | 通过，持续 66.765 秒 |
| TCP 关闭后新建连接，第 1 次 | 30/30 | 16 ms | 通过 |
| TCP 关闭后新建连接，第 2 次 | 30/30 | 16 ms | 通过 |
| 电脑主动断开 Wi-Fi，再重新连接热点 | 30/30 | 109 ms | 通过 |

首轮命令在项目根目录执行：

```powershell
& .\.venv\Scripts\python.exe pc/probe.py --host 192.168.4.1 --port 6001 --allow-hardware --count 30
```

连续及 TCP 重连检查调用相同 probe 实现。成功回复均通过序号、CRC32、固件标识 0x20261007、NO_MOTOR_IMPLEMENTATION 标志及 STM32 计时检查；也检查了不同 TCP 会话之间的计时没有回退。各阶段末尾 STM32 uptime 分别为 152752、270755、274531、278293 ms。

Wi-Fi 恢复检查在已确认真实回复的 TCP 会话上调用 netsh wlan disconnect，再关闭旧 socket，并通过保存的手动配置重新连接 ChassisDrive-Setup。掉线前的 1 次回复 uptime 为 337853 ms；重新连接并完成 30 次回复合计耗时 5.406 秒，最后 uptime 为 344392 ms，未观察到 STM32 计时回退。这个 5.406 秒包含后续 30 次请求，不是单独的 Wi-Fi 关联耗时。

ESP32 的现有网关代码只转发 UART 回复，不自行代答。本轮通信确认来自现有 STM32 诊断固件；没有加入模拟对端或运动代码。

机器可读结果与串口日志保存于 Git 忽略的 logs/2026-10-10-communication/：tcp_checks.json、wifi_reconnect.json、esp32_after_power_cycle.log。没有将 Wi-Fi 配置文件或备份加入 Git。

## 初次检查历史（已被上述成功结果接续）

此前一轮检查中，手机共享已成功，而 ESP32 没有日志或热点；USB 只读识别及复位未使现象恢复。以下保留当时观察，不能当作当前仍然失败。

### 当时的手机网络

用户确认共享已开启。Windows 检测到 Apple Mobile Device Ethernet（以太网 3，接口索引 3）处于 Up，IPv4 为 172.20.10.2，网关为 172.20.10.1。

- 从 172.20.10.2 向手机网关发送两次 ping，两次回复，丢包 0%。
- 使用 curl 显式绑定 172.20.10.2 访问 https://www.baidu.com/，返回 HTTP 200，实际本地地址也是 172.20.10.2。
- IPv4 默认路由选择手机接口；手机接口 metric 25，WLAN metric 35。
- 对 www.microsoft.com 与 1.1.1.1 的本轮连接尝试超时；手机互联网可达性的成功证据是上面的百度 HTTPS 请求。

初次检查时电脑 WLAN 仍连接 HUST_WIRELESS，IPv4 为 10.19.126.146，当时没有切换 Wi-Fi、关闭网卡或修改路由。

### 当时的 ESP32 现象

重新枚举识别到 COM8，VID 303A/PID 1001，序列号 B8:1F:3F:C4:6E:DC，与既有 ESP32 一致；Windows Ports、USB JTAG/serial 与复合设备状态均为 OK。

多次读取 COM8（115200，分别约 12 秒、11 秒、11 秒），均没有收到字节；在不同 DTR 配置下都未观察到周期日志。调用 Windows WlanScan 刷新后，多次扫描未发现 ChassisDrive-Setup；扫描可发现校园网的 2.4 GHz 和 5 GHz 热点。

尝试通过原生 USB 只读识别并复位既有程序：

```powershell
& .\.venv\Scripts\python.exe 'C:\Users\imxiaocai\.platformio\packages\tool-esptoolpy\esptool.py' --chip esp32s3 --port COM8 --baud 115200 --before usb_reset --after hard_reset --no-stub read_mac
```

命令成功退出，识别 ESP32-S3 (QFN56) revision v0.2、8MB Embedded PSRAM、40MHz 晶振、USB-Serial/JTAG、MAC b8:1f:3f:c4:6e:dc，并报告 Hard resetting via RTS pin。没有写入或擦除 Flash，没有修改 eFuse，也没有重烧固件。

复位后再读取约 11 秒仍未收到日志，刷新扫描仍未发现预期热点。因此当时仅确认 USB/ROM 通信可用，尚不能确认诊断应用正常运行；没有证据确定 BOOT 状态、供电、程序启动或其他因素中的哪一项导致当前现象。

## 当前接续点

通信往返、约 67 秒持续运行、TCP 重连和一次 Wi-Fi 掉线恢复已经初步验收。尚未做更长时间运行、UART 物理断线对照、故障注入或整车电气验收；这些不能由本轮成功自动推定。

下一阶段先核对当前 TB6612 模块排针、电机线序及电流、动力电池参数，确定 STM32 PWM/方向/STBY 接线与默认禁用状态，再实现运动指令和失联停车，并架空低速逐轮验证。当前仍没有电机 GPIO/PWM/编码器实现，STOP 仅为诊断操作。

所有报告中的 hardware_verified: false 表示未验收电气、物理停车与整车安全，不表示本轮通信测试失败。历史完整 Flash 备份继续保留。
