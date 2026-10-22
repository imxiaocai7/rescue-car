# 智能救援小车

## 2026年10月22日晚提交的 ESP32 代码

本次新增独立的 [电脑控制底盘工程](software/chassis-drive/README.md)，包含 ESP32-S3 v3 通信网关、配套 STM32 固件、电脑前进/后退/转向/停车控制端，以及协议和测试。版本说明见 [10月22日晚 ESP32 代码](software/chassis-drive/docs/ESP32_2026-10-22_EVENING.md)。35 项软件测试与两板构建通过，尚未完成本次硬件烧录与带动力验收。当前独立工程的通信协议以其配套文件为准。

工创赛智能救援赛道团队项目，包含电脑端视觉与规划、ESP32-S3 图传/控制网关，以及 STM32F407VET6 底盘控制代码。

系统采用两轮差速底盘。电脑经 Wi-Fi 向 ESP32 发送控制帧，ESP32 经 UART 转发给 STM32；STM32 回传状态。OpenMV 图像经另一组 UART 和 TCP 连接传到电脑。

## 当前开发状态

- 电脑端：已有手动底盘控制界面、本机模拟器、零速通信验收工具、视觉数据集处理与规划模块。
- ESP32：已有 OpenMV 图传与电脑/STM32 双向通信网关；相机图传有历史硬件实测记录。
- STM32：已有可构建的 **communication-only 通信验收固件**，只接受零轮速和舵机 HOLD，电机输出保持禁用。
- 比赛识别模型、真实传感器与编码器接入、带动力底盘、相机标定和整车自主救援仍需后续开发、标定及实物验收。

本机模拟、测试通过和固件编译成功各自有独立含义。具体测试日期、硬件前提和限制以各模块说明为准。

## 目录

| 路径 | 内容 |
| --- | --- |
| `software/pc/` | 电脑端界面、通信、视觉、定位、规划和测试 |
| `software/esp32/` | ESP32-S3 网关工程 |
| `software/openmv/` | OpenMV 采图与 UART 图传脚本 |
| `software/stm32/` | STM32 通信验收工程与可复用控制内核 |
| `software/tests/` | C 控制与协议内核的主机测试 |
| `资料/` | 厂商提供的电机、编码器和 TinyF 参考 PDF |
| 根目录方案文档 | 项目规划、接线、控制方案和答辩材料 |

## Windows 上手

克隆本仓库后，在仓库根目录打开 PowerShell。安装具有 Tkinter 的 Python，然后创建各自的本地环境：

```powershell
python -m venv .venv-vision
& .\.venv-vision\Scripts\python.exe -m pip install -r software/pc/requirements-usb.txt
```

运行底盘本机模拟界面：

```powershell
& .\.venv-vision\Scripts\python.exe software/pc/control_chassis.py --demo
```

运行零速通信工具的本机演示：

```powershell
& .\.venv-vision\Scripts\python.exe software/pc/probe_motion.py --demo
```

以上演示使用本机模拟器。真实硬件联调步骤见 [底盘控制说明](software/pc/CHASSIS_CONTROL.md)、[零速通信验收说明](software/pc/MOTION_COMMISSIONING.md) 和 [STM32 工程说明](software/stm32/README.md)。

运行电脑端测试：

```powershell
& .\.venv-vision\Scripts\python.exe -m unittest discover -s software/pc -p "test_*.py"
```

涉及 Python/C 协议兼容的测试还需要本机 GCC，安装与运行方式见 STM32 工程说明。

## 构建 STM32 通信验收固件

```powershell
& .\.venv-vision\Scripts\python.exe -m pip install platformio==6.1.18
& .\.venv-vision\Scripts\python.exe -m platformio run --project-dir software/stm32
& .\.venv-vision\Scripts\python.exe software/stm32/tools/check_firmware.py
```

这些命令生成并检查候选固件。上板前仍须核对 ST-Link、实际引脚、供电与原程序备份，按 STM32 说明完成通信验收。

## 视觉开发

视觉采图与实测记录见 [OpenMV USB 指南](software/openmv/USB_VISION_GUIDE.md) 和 [ESP32 网关说明](software/esp32/README.md)。

训练环境依赖在 `software/pc/requirements-yolo.txt`。比赛类别、数据集目录及标注规范见 [视觉数据集说明](software/pc/VISION_DATASET.md)。训练图片和权重由团队另行共享，克隆代码不会得到这些数据。

## 团队协作

每个人在自己的电脑克隆仓库，并在本仓库设置自己的提交姓名、邮箱。开始修改前执行 `git status`，保持已有修改。

从最新 `main` 建立任务分支，例如 `vision/target-detection`、`control/stm32-uart`。每完成一个独立可运行模块，检查差异、执行相关验证并提交，通过 Pull Request 合并到 `main`。避免多个成员同时直接修改主分支或覆盖他人的提交。

团队本地路径和串口号可能不同，优先从仓库根目录使用相对路径运行，并重新确认硬件端口。部分历史文档保留了作者机器路径。

## 文件共享范围

本仓库提交项目源码、配置、团队方案文档和厂商参考 PDF。虚拟环境、PlatformIO 构建目录、Flash 备份、相机原图、救援训练集、训练产物、PPT 预览缓存，以及 `资料/` 中的压缩包、视频、安装工具和厂商示例工程保留在各自本机；需要时另行共享。

`资料/` 下的 PDF 为对应厂商资料，版权归原作者或厂商所有。
