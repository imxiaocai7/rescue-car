# 第二阶段电脑联调：只发零速

用于验收电脑 TCP → ESP32 透传 → STM32 USART2 → RST1 回传的软件链路。
**本轮没有硬件在身边，没有连接真实端点或烧录。以下默认演示全在本机完成。**

## 没有硬件也能运行

在 `C:\Users\imxiaocai\Desktop\gc` 根目录运行：

```powershell
python software/pc/probe_motion.py --demo
```

默认运行约 3 秒，启动专用本机模拟器、发送左右轮零速 / 舵机 HOLD、等待确认，然后关闭模拟器。
它不会调用串口、ST-Link 或摄像头，也不会连接 Wi-Fi/ESP32。可用 `--duration 10` 调整持续时间。
模拟器的软件锁定与无传感器状态对应第二阶段协议约定，但它不是 MCU/HAL 执行器。

完成后打印 JSON：

- `mode: simulation`：证明这次连接的是本机演示端点，不是小车。
- `received_frames`：有效且递增的状态帧数量。
- `sent_zero_frames` / `acknowledged_zero_frames`：主测试发送轮次及确认数，不包括退出前的最后一次未确认零速尝试。
- `last_confirmed_sequence`：STM32/模拟器确认的最新本轮指令序号。
- `max_ack_round_trip_ms`：发指令到收到确认的最大电脑计时往返值，包含对端遥测调度与网络/系统延迟；不是单向 UART 延迟，也不是视觉延迟。
- `software_interlock_reported: true`：收到软件锁定状态，不是物理电平测量。
- `hardware_verified: false`：此工具不证明引脚、电流、停车或整车安全。

没有反馈、没有指令确认、状态与通信验收版不符、计时回退或序号异常时，输出失败并关闭连接。
不会自动重连、绕过锁定或补发非零动作。重复运行会从最新遥测的确认序号重新同步。

## 与第一阶段界面的区别

第一阶段 `control_chassis.py` 是手动运行入口，必须有有效编码器且无急停才能授权。
第二阶段 STM32 固件还没有编码器采样，且常置软件停机位，因此该界面应当保持锁定。
`probe_motion.py` 只验证**零速指令是否被接收、状态是否回传**，不要求伪造编码器，也不清除停机位。
普通第一阶段模拟器不具有这个验收版状态，本工具会拒绝它。

固件工程、构建命令、候选烧录文件及未来实物检查清单见 [STM32 说明](../stm32/README.md)。

## 未来硬件验收的入口（本轮不运行）

完成 ST-Link、芯片、电源、旧 Flash 备份与烧录核对，且电机动力电断开后，才使用：

```powershell
python software/pc/probe_motion.py --host 192.168.4.1 --port 5001 --allow-hardware --duration 10
```

地址要替换为 ESP32 的实际 IPv4。非本机目标默认拒绝，`--allow-hardware` 只是连接开关，不表示硬件验收通过。
停止其他运动控制客户端；运动端口只供一个客户端使用。不要把图像端口 5000 用作运动诊断。
当前固件只接受零速 / HOLD；状态中停机位代表软件验收锁定，不是已安装物理急停。

不需要更改 OpenMV 接线或刷新 ESP32，但实际 UART、Wi-Fi、供电和长期可靠性仍须实测。
出现无回传时不能假定 TCP 成功即 UART 成功；检查 TX/RX 交叉、共地、波特率、复位、启动模式和固件版本。
RST1 没有独立的固件身份/会话字段，状态匹配不能作为安全认证。

## 软件测试

```powershell
& .\.venv-vision\Scripts\python.exe -m unittest discover -s software/pc -p "test_motion_probe.py" -v
& .\.venv-vision\Scripts\python.exe -m unittest discover -s software/pc -p "test_*.py"
```

本工具使用本机 TCP 模拟器做连接、零速确认、重连、无确认、无回传及错误状态检查。
另有 `test_stm32_commissioning.py` 用本机编译的真正 C 协议核心做 Python↔C 字节兼容检查；
两者分别验证软件，不代替 MCU 中断、实际电平和无线通信测试。

## 软件验证记录（2026-10-03）

- 全量电脑端测试 161 项通过，包含本工具 12 项测试。
- 本工具测试重复 5 轮，每轮 12 项均通过。
- STM32 通信验收固件清理构建产物后重新交叉编译成功，ELF/BIN/HEX 检查通过。
- 默认本机演示成功；以上记录不包含烧录、真实 UART/Wi-Fi 或电机测试。
