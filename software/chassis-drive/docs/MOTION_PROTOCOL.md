# 运动协议 v2 与实现记录

> 2026-10-22：线协议字段保持不变，ESP32 已重写为 v3，运动确认超时改为 150ms（PC 200ms、STM32 租约 300ms），新增网关会话/使能检查。电脑支持鼠标方向按钮和方向键，完整当前约定见 [电脑控制说明](PC_CAR_CONTROL_2026-10-22.md)。STM32 主程序已经移除旧 STBY/PB0 控制；下面 STBY/TB6612 说明及 400ms 运动网关超时保留为历史记录。新固件尚未烧录。

> 2026-10-19 当前驱动改为 L298N，方向引脚改为 PB11–PB14，PWM 仍为 PC6/PC7，PB0 不接驱动。新固件已构建但未烧录；下文早期 STBY/TB6612 验收是历史记录，当前接线与待验收项目见 L298N_WIRING_2026-10-19.md。

2026-10-11：已实现、完成软件测试/交叉编译，并已刷写两板，真实无动力的协议/输出寄存器验收通过；尚未验收电机动作和物理停车，见 [刷写及验证记录](MOTION_BRINGUP_2026-10-11.md)。STM32 运动构建环境是 `stm32-motion`；`stm32-uart-check` 仍保留诊断固件。新 ESP32 桥接同时提供 6001 诊断与 6002 运动端口，两者总共只接受一个活动客户端。

所有多字节整数小端；左右值为有符号 16 位千分比占空比，不是 rpm 或线速度。CRC32/ISO-HDLC，均覆盖 CRC 字段前的全部字节。

| 请求偏移 | 字节数 | 字段 |
|---|---|---|
| 0 | 4 | DCM2 |
| 4 | 4 | sequence |
| 8 | 4 | session，PC 每次连接随机非零 |
| 12 | 1 | HELLO=1、ARM=2、DRIVE=3、STOP=4、STATUS=5 |
| 13 | 3 | 保留零 |
| 16 | 4 | token |
| 20 | 2 | left duty，-250..250 |
| 22 | 2 | right duty，-250..250 |
| 24 | 4 | 保留零 |
| 28 | 4 | CRC |

| 回复偏移 | 字节数 | 字段 |
|---|---|---|
| 0 | 4 | DCS2 |
| 4 | 4 | 请求 sequence |
| 8 | 4 | 请求 session |
| 12 | 4 | STM32 uptime_ms |
| 16 | 4 | 固件 ID，0x20261011 |
| 20 | 4 | flags：ARMED=1、TIMED_OUT=2、UART_FAULT=4 |
| 24 | 4 | 当前 token，禁用后为零 |
| 28 | 2 | left 当前斜坡输出占空比 |
| 30 | 2 | right 当前斜坡输出占空比 |
| 32 | 1 | result：OK=0、BAD_SESSION=1、OLD_SEQUENCE=2、NOT_ARMED=3、BAD_ARGUMENT=4 |
| 33 | 3 | 保留零 |
| 36 | 4 | CRC |

## 启停及租约

- HELLO 的 token 和占空比均为零，先禁用输出并获得新 token。token 基于 STM32 硬件 RNG 提供的本次启动随机种子，每次 HELLO 更换。RNG 不可用则启动失败并保持禁用。这是避免旧控制会话残留的握手，不是身份认证。
- ARM 必须带匹配 session/token、新 sequence，且左右均零。已使能时不能重复 ARM 延长租约。
- DRIVE 仅在已使能时接受，左右限定 ±250（±25%），每个接受的运动请求更新 300 ms 命令租约。硬件适配层以最后一个请求字节的 UART 收到时间计租约，避免处理队列延迟额外延长。
- STATUS 不延长运动租约。重复序号、旧序号、错误 session/token、超范围参数和损坏帧也不延长。序号采用半范围回绕比较。
- STOP 只要帧有效即可禁用，不要求旧会话仍有效；PC 发送零占空比。停止清除 token，下一次运行必须 HELLO 后重新 ARM。
- STM32 SysTick 每毫秒检查租约；超时、UART 错误/队列溢出、过期队列数据及致命错误撤销 STBY/PWM。看门狗复位不在运动构建中因 SWD 调试暂停而冻结。2026-10-19 新板图确认 F4VE 的 PB0 另有 R20=10kΩ 上拉至 3V3，不能只依靠 TB6612 内部 200kΩ 下拉维持复位期禁用；当前接线方案要求在 STBY 对地增加 1kΩ 下拉，保留 PB0，见 POWER_AND_WIRING_2026-10-19.md。实际启动/复位电平仍待验收，固件运行态验证不能替代该检查。
- 每 10 ms 输出向目标靠近最多 20 千分比；反向先降至零，等待 20 ms，再向相反方向增加。较迟调度不会一次补齐多个斜坡步长。
- 禁用采用 STBY 低、高阻滑行；撤销输出不是对物理停稳时间的承诺。ARM 后零占空比状态不能被当成已验收的停车策略。

## 网关与电脑

ESP32 转发真实 STM32 回复，不合成使能/停车成功。客户端断开、超时及新连接时，网关尝试发送无需追踪的 STOP；该动作失败时依赖 STM32 本地租约。网关只允许一个待确认请求，超时 400 ms，序号/session/固件不匹配时关闭连接，旧会话的 STOP 回复不能确认新 PC 请求。

PC 命令超时默认 200 ms，运动循环约每 50 ms 更新。错误回复、STM32 计时回退、超时或断线会结束当前控制；不自动重连或使能。键盘程序连接后保持禁用，需点击使能；松开运动键目标归零，空格、窗口失焦和退出禁用输出。未测得轮子实际方向前不得直接落地双轮运行。

原诊断 v1 的格式保留：运动固件以 ID `0x20261011`、flags=2 表示已包含电机实现；诊断 STOP 也禁用电机。旧诊断固件仍返回原 ID 与未实现标志。`pc/probe.py` 与 ESP32 诊断网关接受这两种已知组合，但通信确认仍不证明实物停车。

## 构建与 PC 入口

在项目根目录：

```powershell
& .\.venv\Scripts\python.exe -m unittest discover -s tests -v
& .\.venv\Scripts\python.exe -m platformio run -d firmware/stm32 -e stm32-motion
& .\.venv\Scripts\python.exe tools/check_stm32_artifact.py --environment stm32-motion
& .\.venv\Scripts\python.exe -m platformio run -d firmware/esp32
```

以上均不上传。运动 HEX 在 `firmware/stm32/.pio/build/stm32-motion/firmware.hex`。上传前需要动力断开、ST-Link 和目标核对；旧完整备份保留。

刷写后的只读状态入口（HELLO/STOP 会确保禁用，不会 ARM）：

```powershell
& .\.venv\Scripts\python.exe pc/drive.py --host 192.168.4.1 --allow-hardware
```

架空单轮短时测试，下面示例左轮 10%、0.5 秒，右轮零；执行前要完成不接动力的输出验收：

```powershell
& .\.venv\Scripts\python.exe pc/drive.py --host 192.168.4.1 --allow-hardware --allow-motion --pulse --left 100 --right 0 --duration 0.5
```

双轮架空、方向校准后启动键盘窗口：

```powershell
& .\.venv\Scripts\python.exe pc/drive.py --host 192.168.4.1 --allow-hardware --allow-motion --keyboard --duty 100
```

以上动力单轮命令与键盘运行仍待进行。本轮只在动力断开时用真实协议验证输出寄存器和停机状态。低占空比只用于短时起转检查，不等于限流；不转时停止排查。

## 软件验证与剩余项目

31 项测试通过，新增测试直接编译/执行固件使用的 C 协议和控制核心，覆盖超时、重复/旧序号、旧 token、换会话、超范围命令、不续租的状态查询、时间与序号回绕、反向斜坡及 UART 故障禁用状态。另验证 Python/C 帧兼容、PC 分片回复与错误确认、键盘归零/禁用策略和显式运动开关。

STM32 两个构建和 ESP32 构建通过。软件验证不包含实际 PWM 频率/电平、GPIO 启动时序、网络/串口失联撤销输出延迟、温升、实物轮向或停车距离。编码器、速度闭环和随车低压供电尚未实现。
