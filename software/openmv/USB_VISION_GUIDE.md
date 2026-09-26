# OpenMV USB 视觉联调

这一入口只连接 OpenMV 的 USB 串口，不连接 ESP32 或 STM32，不发送运动指令。
板上 `main.py` 和固件不被改写。先跑通 USB 图像、采图和标定，再联调 Wi-Fi。

## 已核实的设备

2026-09-26 本机实测：串口 `COM10`，USB VID:PID `37C5:124A`；固件 `4.7.0`，
板卡报告 `OMV4P H7 32768 SDRAM`，MicroPython 日志报告 `OPENMV4P with STM32H743`，
传感器 ID 为 `0x56`。这些是固件返回值，不是根据商品图片推断的硬件型号。
COM 号换电脑或换 USB 接口后可能改变。

板载盘 `E:` 曾被 Windows 标记为需要修复，不能因此认定串口驱动有问题。
未格式化、未修复磁盘、未刷固件。盘根目录现有脚本和临时脚本已经备份到本机
`local_backups/openmv_20260926_230631/`，未纳入 Git。
原始 `main.py` SHA-256：
`7C7BEE2735D37595E3B529F77385F077804D24C53C86718A8AF86674C7C9A28E`。

## 启动

在仓库根目录运行。当前机器已准备好 `.venv-vision` 环境，后续使用它的 Python，
不要混用系统 Python。新电脑可用以下命令建立独立环境：

```powershell
python -m venv .venv-vision
.\.venv-vision\Scripts\python.exe -m pip install -r software/pc/requirements-usb.txt
.\.venv-vision\Scripts\python.exe software/pc/view_openmv_usb.py --list
.\.venv-vision\Scripts\python.exe software/pc/view_openmv_usb.py --port COM10 --probe
```

`--list` 不打开任何串口；`--probe` 只查询版本与板卡信息，不停止现有脚本。
本客户端使用 OpenMV 3.x/4.x 的旧 USBDBG 协议，不保证兼容其他固件。
协议核对来源：[OpenMV 官方 v4.5.9 Python 客户端](https://github.com/openmv/openmv/blob/v4.5.9/tools/pyopenmv.py)。
不能把这个接口当普通 UVC 摄像头使用，也不能让 OpenMV IDE 与本程序同时占用串口。

```powershell
.\.venv-vision\Scripts\python.exe software/pc/view_openmv_usb.py --port COM10 --start-preview --save-dir dataset/camera_raw --notes "桌面采集；固定镜头；环境光；目标与距离待记录"
```

- `--start-preview` 明确停止当前运行脚本，把仓库的 `usb_preview.py` 临时送入 RAM 运行。
  不保存到板载盘，不替换启动脚本。退出时停止此次临时脚本；重新上电恢复原来的启动程序。
- 初始图像为 RGB565/QVGA（320×240）；USB framebuffer 通常以 JPEG 传输。
  自动曝光/白平衡预热结束、收到就绪日志之后，才计数或采集。
- 按 **S** 保存无画框、无文字的原始图像；按 **Q/Esc** 或关闭窗口结束。
  JPEG 保留收到的原始字节，灰度/RGB565 原始帧转无损 PNG。
- 不加 `--start-preview` 则读取已运行脚本的 framebuffer，该脚本可能自带标记，不适合作训练原图；
  此模式退出时不会停止原脚本。
- 不加载模型时明确显示 `PREVIEW ONLY`。已有救援模型时才加 `--model 本地权重.pt --device 0`，
  并另装 `requirements-yolo.txt`；USB 连通不代表完成救援识别。

无窗口稳定性检查（仅做台架采图，不是训练集）：

```powershell
.\.venv-vision\Scripts\python.exe software/pc/view_openmv_usb.py --port COM10 --start-preview --headless --frames 120 --duration 30 --save-dir dataset/camera_raw --save-every 30 --notes "USB稳定性检查；未标定"
```

每次运行新建独立场次目录，避免帧号重启覆盖旧图：

```text
dataset/camera_raw/<UTC时间_随机后缀>/
  session.json    # 固件、设备、备注、采集脚本、未标注状态
  frames.jsonl   # 图片尺寸、接收时间、格式、哈希、USB读取耗时
  frame_*.jpg    # 或 PNG，仅在按 S 或指定 save-every 时保存
  summary.json   # 帧数、接收帧率、读取耗时、板上日志、错误
```

图片、备份、环境不提交 Git，也不自动上传。`--notes` 应记录光照、安装角度、场景、拍摄距离。
相邻帧必须按整个采集场次分入同一 train/val/test 集，不能随机拆散造成数据泄漏。
没有比赛目标的调试照片不能直接当作有效救援训练数据；见
[`../pc/VISION_DATASET.md`](../pc/VISION_DATASET.md)。

## 测量含义与实测记录

预热后 120 帧的台架测试：320×240 JPEG，平均接收 23.29 FPS，平均 USB 读取 23.33 ms，
最大读取 24.98 ms，无客户端错误。保存了 4 张图，记录位于本机
`dataset/camera_raw/20260926T151043Z_aa8cd69a/summary.json`。

随后带窗口预览成功读取 213 帧（平均约 19.14 FPS），发生 USB 写超时；同一时刻
Windows 的 OPENMV 盘消失。操作者随后确认当时移动过相机，并且已经看到预览画面；
可能牵动了数据线，具体物理原因未进一步确认。异常已保留在该场次 `summary.json` 中。
断连时不自动重连或重启脚本，
重新连接后显式启动；若停止命令无法发送，会明确提示，不能保证断连板卡的 RAM 脚本已停止。

重新连接后又完成 30 秒测试（含预热）：628 帧、320×240、平均 23.30 FPS，平均 USB 读取
21.20 ms、最大 27.15 ms，无客户端/清理错误，退出时已发送停止 RAM 脚本命令。
此次没有保存图片；日志位于 `logs/vision/20260926T151824Z_81a0d7bc/summary.json`。
重连后再次核对 `main.py` 哈希，与备份一致。上述短测不替代长时间/线束固定后的稳定性验收。

软件验证：新增 USB/采图 23 项测试及原有图像协议/视觉辅助 10 项测试全部通过。
全量 102 项测试中仍有 1 项本次改动前就存在的任务策略失败：
`test_autonomy_cycle.AutonomyCycleTests.test_no_legal_target_and_missing_calibration_stop`。
该问题不在此次相机改动中处理；不能据此宣称自主决策全套回归已通过。

这个帧率不是保证值，受光照、分辨率、USB 和电脑负载影响。USB 读取耗时包含读帧/解码开销，
**不是**传感器曝光到电脑的端到端延迟。USBDBG 不提供本程序所需的相机曝光时间戳，
记录中的 `camera_ticks_ms` 为 null；不要拿电脑时间当相机时间做运动融合。
USB framebuffer 压缩也不等于 ESP32 的 QUALITY=40 管线；最终须用 Wi-Fi 图像再次验证模型和延迟。

## 常见故障

- 找不到设备：检查数据线、USB 接口、设备管理器；不要凭“电源灯亮”判断数据连接正常。
- `Access denied`：关闭 IDE/其他串口程序，确认选择正确 COM 口后重试。
- 报 `cvNamedWindow ... function is not implemented`：当前 Python 导入的是 headless OpenCV，
  使用 `.venv-vision` 内的 `opencv-python`（带 Windows GUI），不要删除其他项目的全局依赖。
- 有串口但无帧：查看脚本日志，使用 `--start-preview`；无图超过 10 秒会停止并报错。
- 固件版本不支持：保留固件和启动脚本，先核对兼容协议，不为取图盲目升级固件。
- 画面暗、倾斜或物体太小：先固定相机朝向和距离、补充均匀光线；不要先靠算法掩盖问题。

## 下一阶段

1. 固定相机，拍摄实际普通物资、核心物资、伤员、危险目标及空场景，核对颜色/清晰度。
2. 采集多场次数据并按固定五类人工标注、审查；没有实物样本和标注，不宣称识别已完成。
3. 在最终镜头、分辨率、安装高度/角度下采棋盘格和地面对应点，完成内参与地面投影标定。
4. 训练和评估救援模型，再验证 ESP32 图传与推理；未标定、未验证的框不接入运动控制。
