# 底盘手动控制：第一阶段软件

本阶段使用现有 `RCM1` 左右轮速度指令和 `RST1` 遥测协议，不修改或烧录板子。
模拟器只验证协议、序号与通信失效行为，不模拟真实 PI、电机电流、惯性、制动距离或轮胎打滑。

## 独立运行本地模拟器

在仓库根目录运行：

```powershell
python software/pc/chassis_simulator.py
```

服务只监听 `127.0.0.1:5501`，不访问串口、ESP32 或任何硬件。按 Ctrl+C 退出。
每 50ms 输出一帧遥测；有效命令租约到期后归零，断开 TCP 不重置命令序号。
默认 75mm 轮径、1560 计数/圈仅用于合成编码器数据，**不是实际轮径标定**。
没有陀螺、测距或电池采样，相关字段标为无效。

故障注入示例（时间从模拟器启动算起）：

```powershell
python software/pc/chassis_simulator.py --drop-commands
python software/pc/chassis_simulator.py --mute-telemetry-after 10
python software/pc/chassis_simulator.py --emergency-stop-after 10
```

测试：

```powershell
python -m unittest discover -s software/pc -p "test_chassis_simulator.py" -v
```

这是独立可运行的协议模拟模块；电脑控制界面将在下一模块接入。
