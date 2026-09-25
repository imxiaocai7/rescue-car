# 第一阶段软件：相机图像帧协议

这部分只处理图像，不会向电机或舵机发命令。电脑端帧编解码、坏帧重同步、OpenMV 采集脚本、ESP32-S3 网关和预览程序已写好，但整条链路**尚未在实物上运行验证**。

## 数据流

ESP32 向 OpenMV UART3 发送单字节 `R`，OpenMV 才采集并发送一帧。ESP32 收齐、校验后，把原始帧数据不修改地写入电脑 TCP 连接（端口 5000）；电脑处理并显示后，回发帧号确认。ESP32 收到匹配确认才请求下一帧，视觉推理自然形成背压。电脑端按以下小端格式解析：

| 字段 | 字节数 | 值 |
|---|---:|---|
| 魔数 | 4 | `RSV1` |
| 版本、类型 | 1+1 | `1`、`1`（JPEG） |
| 帧序号、相机开机毫秒数 | 4+4 | 无符号 32 位 |
| 宽、高 | 2+2 | 无符号 16 位 |
| JPEG 长度、CRC32 | 4+4 | CRC32 只覆盖 JPEG 字节 |
| JPEG | 可变 | 不大于 120000 字节 |

电脑端解析器对 TCP 分块、粘包、前导噪声、异常长度和坏 CRC 做了恢复。CRC 使用标准 IEEE CRC-32，与 Python `zlib.crc32` 一致。这里的相机毫秒数尚未与电脑时钟同步，不能直接拿它计算端到端绝对延迟；后续用握手往返和电脑接收时间建立估计。

ESP32-S3 网关代码及连线、构建说明见 [`esp32/README.md`](esp32/README.md)。当前网关按照片中的 ESP32-S3-N16R8 配置，启动 Wi-Fi AP `RescueBot-S3`，TCP 服务为 `192.168.4.1:5000`；不包含 STM32 电机控制。

TCP 每帧后电脑要回发 `ACK1 + 帧序号(u32小端)`，并在处理完该帧后再确认。网关收到匹配确认才向 OpenMV 请求下一帧，因而 YOLO 推理速度会自然限制相机输出，降低控制使用陈旧图像的风险。

电脑端实时预览及可选 YOLO 推理见 [`pc/view_camera.py`](pc/view_camera.py)。没有训练权重时可只看预览和采集 JPEG；加载权重后才显示模型检测框。当前仓库没有救援类别训练集或权重，因此不应把通用模型检测结果用于自主决策。

离线目标规则模块见 [`pc/mission_policy.py`](pc/mission_policy.py)：首个任务仅允许收集一个普通物资；危险物和对方机器人仅作为障碍；伤员单独运输；普通与核心物资总数封顶 3 个。输入必须是已完成分类、置信度筛选和地面坐标投影的候选目标，以及确认后的任务/载荷状态；模块只返回合法目标，不驱动车辆。

离线 A* 栅格规划器见 [`pc/grid_planner.py`](pc/grid_planner.py)：默认地图为 3 m × 3 m、50 mm 分辨率，可配置矩形/圆形障碍、机器人安全膨胀和场地边界余量；使用 8 邻域且禁止斜穿障碍角。它只生成地图路径点，不生成轮速或底盘动作，实际膨胀距离须待 CAD 外形和轮距冻结后设置。

将目标合法性和路线可达性联合排序的离线决策入口见 [`pc/autonomous_planner.py`](pc/autonomous_planner.py)。目前规划到目标所在栅格中心；真实收集还需要按 CAD 几何确定接近点/朝向，并将检测目标可靠投影到地图。

差速轮编码器里程计与陀螺融合见 [`pc/odometry.py`](pc/odometry.py)：支持计数器回绕、静止陀螺零偏均值估计和中点积分，输入单位为米、弧度/秒。每轮有效半径、轮距、轮子每转计数均需实测后传入配置；当前代码只用合成数据验证，不代表实车定位精度。

路径速度跟踪见 [`pc/path_follower.py`](pc/path_follower.py)：使用纯追踪把 A* 路径点转换为差速底盘的线速度/角速度，目标位于车后方时先原地转向，并限制角速度；另提供按轮距换算左右轮线速度和共同限幅。前视距离、速度上限、轮距都必须结合实车标定。该模块只产生命令数据，不负责 PWM、电机驱动或 STM32 通信，尚未接入硬件闭环。

在本目录运行电脑端测试：

```powershell
python -m unittest discover -s pc -p "test_*.py" -v
```

在仓库根目录启动实时预览（首次先安装 OpenCV）：

```powershell
python -m pip install -r software/pc/requirements-vision.txt
python software/pc/view_camera.py 192.168.4.1 --save-dir dataset/camera_raw
```

按 `S` 可将当前 JPEG 存入指定目录；没有训练权重时只预览/采集，不会产生目标类别判断。准备好符合比赛实物的标注数据和 YOLO 权重后，再安装 `python -m pip install -r software/pc/requirements-yolo.txt` 并运行 `python software/pc/view_camera.py 192.168.4.1 --model runs/detect/train/weights/best.pt --device 0`。不可用通用 COCO 权重替代救援类别权重，也不要将未验证的检测框接入运动控制。

硬件联调时先分别通过 USB 对 OpenMV 和 ESP32 做 115200 bit/s 文本收发，再共地、交叉连接 P4→GPIO16、P5←GPIO15。只有文本和短二进制包稳定后，才在 OpenMV IDE 中运行脚本，并检查帧率、每帧字节数、丢帧和两板温升。相机脚本的 CRC32 是兼容性优先的纯 Python 实现；如果耗时明显，可在实物上换成 OpenMV 固件提供的硬件/本地 CRC 接口。
