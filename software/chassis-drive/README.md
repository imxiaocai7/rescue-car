# 双电机差速底盘驱动（独立新项目）

当前 ESP32 版本：[2026年10月22日晚提交的 ESP32 代码](docs/ESP32_2026-10-22_EVENING.md)，标签 `esp32-2026-10-22-evening`。

建立日期：2026-10-07。本目录使用独立Git仓库，从零实现，不依赖或修改旧项目。

范围：电脑Wi-Fi → ESP32-S3-N16R8 → STM32F407VET6 → 后续双电机差速底盘。
不连接摄像头、舵机或其他传感器。通信已初步实测；2026-10-11 新增开环电机控制，已软件验证、刷写及无动力检查，实际电机运动仍待验收。

新对话接续请先读 [底盘驱动交接档案](docs/CHASSIS_HANDOFF.md)，其中集中记录硬件型号、当前接线、烧录实测、待确认参数和下一步工作。

2026-10-22：已重写 ESP32 v3 通信网关并配套电脑方向控制窗口。双击 `pc/control_car.cmd`，连接后需手动使能，支持方向按钮、W/A/S/D、方向键和停车。完整链路、协议、接线和烧录步骤见 [电脑控制说明](docs/PC_CAR_CONTROL_2026-10-22.md)。新版已通过 35 项软件测试及交叉编译，尚未烧录；STM32 读保护问题仍待处理，实车动作尚未验收。

## 无硬件运行

```powershell
python pc/probe.py --demo
python -m unittest discover -s tests -v
```

默认演示只连接本机模拟器。测试中的C兼容检查需要PATH中有GCC。
`hardware_verified: false` 始终明确表示本工具不证明接线、电平、停车或整车安全。

## 独立开发环境

```powershell
python -m venv .venv
& .\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

源代码、协议、构建配置和测试在本项目中重新建立。可使用全局官方工具包缓存，旧项目代码和环境不参与构建。
协议见 [docs/PROTOCOL.md](docs/PROTOCOL.md)，STM32固件构建见 [docs/STM32.md](docs/STM32.md)。
ESP32桥接见 [docs/ESP32.md](docs/ESP32.md)，首次接线与烧录顺序见
[docs/WIRING_AND_FLASH.md](docs/WIRING_AND_FLASH.md)。STM32和ESP32固件均已交叉编译。
ESP32已备份、烧录校验并启动，记录见 [docs/ESP32_BRINGUP_2026-10-07.md](docs/ESP32_BRINGUP_2026-10-07.md)。
STM32也已备份、烧录校验并启动，记录见 [docs/STM32_BRINGUP_2026-10-07.md](docs/STM32_BRINGUP_2026-10-07.md)。
2026-10-10：Wi-Fi到STM32的真实往返已初步验收，首轮30次、连续600次、TCP重连及Wi-Fi掉线恢复均通过。
详细结果见 [通信链路实物验收](docs/COMMUNICATION_CHECK_2026-10-10.md)。

2026-10-11：用户确认使用独立 TB6612 模块。新的 [逐线接线表](docs/MOTOR_WIRING_2026-10-11.md) 与
[运动协议/台架工具](docs/MOTION_PROTOCOL.md) 已准备；31 项测试、两板构建通过。
运动固件必须显式选择 `stm32-motion`，诊断环境 `stm32-uart-check` 保留。
两板已运行新固件，真实回传与无动力启停/超时检查通过，详见 [运动刷写记录](docs/MOTION_BRINGUP_2026-10-11.md)。

## 安全边界

STM32板卡完整电路、ST-Link输出能力、电机六芯线序、有效轮径、轮距、电池满电电压和驱动电流余量仍需核对。
新接线和刷写先断开动力电池，按逐线接线表进行逻辑检查；之后再架空逐轮验收。
程序的通信确认不代表物理停车能力；未来加入电机控制前必须独立验收。
