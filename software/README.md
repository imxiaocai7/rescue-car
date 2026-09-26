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

比赛目标类别与标注规则见 [`pc/VISION_DATASET.md`](pc/VISION_DATASET.md)，固定类别编号和初赛外观参考见 [`pc/rescue_classes.json`](pc/rescue_classes.json)：普通物资、核心物资、伤员、危险目标、对方机器人。数据集审查、训练和评估入口为 [`pc/vision_workflow.py`](pc/vision_workflow.py)，支持 YOLO 标签校验、漏标/孤立标签检查、类别分布统计、损坏图片和跨集合完全重复图片检查。数据集按文档存放在 `dataset/rescue`；已加入本机 Git 忽略规则，避免提交图片、模型权重和训练产物。

在仓库根目录运行 `python software/pc/vision_workflow.py audit --dataset-root dataset/rescue` 审查；训练使用 `python software/pc/vision_workflow.py train --dataset-root dataset/rescue --model yolo26n.pt --device 0`；最终留出的测试集评估使用 `python software/pc/vision_workflow.py evaluate --dataset-root dataset/rescue --weights runs/rescue/rescue/weights/best.pt --split test --device 0`。初始参数按 Windows 单张 RTX 4060 的小批量设置，数据和训练运行尚不存在时不会产生模型或识别效果结论。

离线目标规则模块见 [`pc/mission_policy.py`](pc/mission_policy.py)：首趟仅允许收集一个普通物资；完成首趟后，普通与核心物资按当前趟载荷计数、每趟最多 3 个；伤员单独运输；危险物和对方机器人仅作为障碍。累计已交付物资只用于任务进度，不占用下一趟容量。输入必须是已完成分类、置信度筛选和地面坐标投影的候选目标，以及确认后的任务/载荷状态；模块只返回合法目标，不驱动车辆。

完整任务执行状态机见 [`pc/mission_executor.py`](pc/mission_executor.py)：连接搜寻、导航到目标、等待收集确认、返航和等待投放确认；按照物资区/伤员区分别规划返程。只有收到外部收集/投放确认事件才更新任务进度；危险或未知障碍进入临时地图；视觉/位姿过期、目标暂时不可见或路线不可达时输出零速帧。该模块消费已经投影到场地坐标的目标，目前收集/投放确认需要仿真器、传感器或上层程序提供，舵机角度、目标间距和安全区坐标仍须结合比赛场地及实物标定。

相机检测到完整任务指令的适配层见 [`pc/mission_runtime.py`](pc/mission_runtime.py)：按配置把类别编号映射为任务类别，将检测框底边中心通过单应矩阵投到场地坐标，并以最近邻关联维持短时稳定目标 ID；危险、未知和低置信度目标作为临时障碍，另可传入已换算到场地坐标的测距障碍。`RescueMissionRuntime.step()` 返回状态机结果和可发给 STM32 的命令帧。该层尚未接入实时 TCP 相机循环，原因是相机外参、目标尺寸和 ToF 安装朝向需按实车测定；目标跟踪目前只适合短时遮挡桥接。

无硬件的端到端任务仿真和逐帧重放见 [`pc/mission_simulator.py`](pc/mission_simulator.py)。默认场景依次模拟首件普通物资、后续三件物资和单独伤员转运，含固定障碍、动态危险圆形障碍、理想差速运动学、传感器延迟/丢帧和运动命令丢帧选项。运行 `python software/pc/mission_simulator.py --record logs/mission.jsonl` 可保存输入、状态和命令帧；运行 `python software/pc/mission_simulator.py --replay logs/mission.jsonl` 会重新执行状态机并逐帧核对输出。可用 `--drop-vision-every 10`、`--drop-odometry-every 10` 或 `--drop-command-every 10` 注入周期丢帧。仿真里的定位和收集/投放确认是理想值，不用于推断实车成功率、避障距离或控制参数。

离线 A* 栅格规划器见 [`pc/grid_planner.py`](pc/grid_planner.py)：默认地图为 3 m × 3 m、50 mm 分辨率，可配置矩形/圆形障碍、机器人安全膨胀和场地边界余量；使用 8 邻域且禁止斜穿障碍角。它只生成地图路径点，不生成轮速或底盘动作，实际膨胀距离须待 CAD 外形和轮距冻结后设置。

将目标合法性和路线可达性联合排序的离线决策入口见 [`pc/autonomous_planner.py`](pc/autonomous_planner.py)。目前规划到目标所在栅格中心；真实收集还需要按 CAD 几何确定接近点/朝向，并将检测目标可靠投影到地图。

差速轮编码器里程计与陀螺融合见 [`pc/odometry.py`](pc/odometry.py)：支持计数器回绕、静止陀螺零偏均值估计和中点积分，输入单位为米、弧度/秒。每轮有效半径、轮距、轮子每转计数均需实测后传入配置；当前代码只用合成数据验证，不代表实车定位精度。

路径速度跟踪见 [`pc/path_follower.py`](pc/path_follower.py)：使用纯追踪把 A* 路径点转换为差速底盘的线速度/角速度，较大航向误差时先原地对准，并将前视目标限制在下一个路径拐点以减小切角；另提供按轮距换算左右轮线速度和共同限幅。`pc/test_navigation_integration.py` 用合成差速运动学验证其能绕开膨胀障碍格。前视距离、速度上限、轮距仍须结合实车标定，仿真不代表实车闭环安全。该模块只产生命令数据，不负责 PWM、电机驱动或 STM32 通信。

电脑规划端到 ESP32-S3/STM32 的速度命令帧和失联租约约定见 [`MOTION_PROTOCOL.md`](MOTION_PROTOCOL.md)，编解码与序号/超时工具见 [`pc/motion_protocol.py`](pc/motion_protocol.py)。这是待固件实现的协议契约，尚未在 ESP32、STM32 实物上联调。

运动链路回传部分已补上：[`pc/rover_telemetry.py`](pc/rover_telemetry.py) 定义 `RST1` 遥测帧和 TCP 分片解码， [`pc/motion_link.py`](pc/motion_link.py) 提供电脑连接 ESP32 端口 5001、发送 `RCM1` 速度帧并接收 STM32 遥测的客户端；STM32 侧编码器在 [`stm32/src/rover_telemetry.c`](stm32/src/rover_telemetry.c)。ESP32 通过独立 FreeRTOS 任务在 TCP 5001 与 UART2 间双向透传，避免阻塞相机端口 5000。UART GPIO 17/18 与 STM32 USART2 PA2/PA3 仅是待核对的候选线序，当前没有 STM32 HAL 采样/发送任务，命令帧也尚未上实物链路。

STM32 可复用的 HAL 无关 C 解码器和序号/租约命令门控位于 [`stm32/include/motion_protocol.h`](stm32/include/motion_protocol.h)、[`stm32/src/motion_protocol.c`](stm32/src/motion_protocol.c) 与 [`stm32/include/motion_command_guard.h`](stm32/include/motion_command_guard.h)、[`stm32/src/motion_command_guard.c`](stm32/src/motion_command_guard.c)。在仓库根目录可用 `gcc -std=c11 -Wall -Wextra -Werror -pedantic -I software/stm32/include software/stm32/src/motion_protocol.c software/stm32/src/motion_command_guard.c software/tests/motion_protocol_test.c -o "$env:TEMP\motion_protocol_test.exe"; & "$env:TEMP\motion_protocol_test.exe"` 做本机编译测试。租约超时门控输出零轮速与 HOLD；仍未连接 UART、PWM、编码器或舵机。

HAL 无关的轮速 PI 控制内核位于 [`stm32/include/wheel_speed_controller.h`](stm32/include/wheel_speed_controller.h) 与 [`stm32/src/wheel_speed_controller.c`](stm32/src/wheel_speed_controller.c)，支持双轮各实例化一份、目标加速度限制、占空比限幅、积分抗饱和与立即禁用清零。独立主机测试命令：`gcc -std=c11 -Wall -Wextra -Werror -pedantic -I software/stm32/include software/stm32/src/wheel_speed_controller.c software/tests/wheel_speed_controller_test.c -lm -o "$env:TEMP\wheel_speed_controller_test.exe"; & "$env:TEMP\wheel_speed_controller_test.exe"`。这些测试只验证数值逻辑；不能据此设定实车 PI 增益或电机安全电流。

双轮命令到控制输出的 HAL 无关协调层位于 [`stm32/include/motion_control.h`](stm32/include/motion_control.h) 和 [`stm32/src/motion_control.c`](stm32/src/motion_control.c)。它串起命令新鲜度/序号检查与双轮 PI 输出；主机测试命令：`gcc -std=c11 -Wall -Wextra -Werror -pedantic -I software/stm32/include software/stm32/src/motion_protocol.c software/stm32/src/motion_command_guard.c software/stm32/src/wheel_speed_controller.c software/stm32/src/motion_control.c software/tests/motion_control_test.c -lm -o "$env:TEMP\motion_control_test.exe"; & "$env:TEMP\motion_control_test.exe"`。此输出仍只是数值接口，不代表引脚、TB6612 电流余量或真实电机动态已验证。

霍尔编码器计数到有符号轮速估计器位于 [`stm32/include/encoder_speed_estimator.h`](stm32/include/encoder_speed_estimator.h) 和 [`stm32/src/encoder_speed_estimator.c`](stm32/src/encoder_speed_estimator.c)。`counts_per_wheel_revolution`、计数器模数、轮半径和最高速度都需按实际定时器解码方式配置；主机测试命令：`gcc -std=c11 -Wall -Wextra -Werror -pedantic -I software/stm32/include software/stm32/src/encoder_speed_estimator.c software/tests/encoder_speed_estimator_test.c -lm -o "$env:TEMP\encoder_speed_estimator_test.exe"; & "$env:TEMP\encoder_speed_estimator_test.exe"`。

像素检测框投影到机器人地面坐标见 [`pc/pixel_ground.py`](pc/pixel_ground.py)：四个地面标定点求平面单应矩阵，检测框底边中心作为落地点，再按差速里程计位姿变换到场地坐标。标定坐标按机器人 x 前、y 左、角度逆时针；相机必须固定，标定点和被投影点需在同一平面。四点投影只是几何初值，镜头畸变和安装变化需靠实测验证，未标定前不可用于自主运动。

单周期 PC 自主决策协调入口见 [`pc/autonomy_cycle.py`](pc/autonomy_cycle.py)：对齐新鲜视觉和里程计时间戳后，将检测框投影到场地、把危险物/对手机器人及未知类别加入临时障碍层，再按任务规则和 A* 可达性选目标，跟踪路径并编码速度帧。类别映射、障碍尺寸/膨胀、目标停车距离、轮距、速度限制和数据新鲜度阈值均要求调用方明确配置；急停、过期输入、无标定、非法框或无合法可达目标会输出零速帧。它仍不负责真正的 YOLO 模型、目标跟踪、全局重定位、拾取动作或硬件通信；坐标是基于当前里程计位姿的估计值。

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
