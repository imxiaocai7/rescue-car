# 运动固件刷写与无动力验证记录

日期：2026-10-11。用户明确确认 12 V 动力与 TB6612 已断开；后续两板 USB 及 ST-Link 均接入电脑。下述验证保持动力断开，不代表电机、接线电平或物理停车已验收。

## 目标、备份与刷写

STM32：F407，DBGMCU_IDCODE `0x10076413`、Flash 512 KiB、UID 三字 `00360017 3138510f 35303737`；ST-Link V2J46S7。读取保护/Option Bytes `0x0fffaaed`，本轮未修改。

当前诊断程序完整备份为 `backups/stm32f407ve_diagnostic_pre_motion_2026-10-11.bin`，524288 字节，SHA256 `7b3872cdae600861a8a4280d069eb9ca7c0ebd0e1c1b2886ff485bd288cb18b5`。前 6912 字节与此前已校验的诊断 BIN 完全一致。完整备份与实物比较成功。OpenOCD 在运行态尝试 CRC 算法时先报告 target not halted，随后软件比较完成 verified 524288 bytes；该提示不是最终备份失败。

新 STM32 构建 `stm32-motion`：BIN 10008 字节，SHA256 `f34413eee8c12377af14e1e90e29ffa5fa5c7cb378f638dfd6c54989c7b9c322`。使用 HEX 写入，输出 Programming Finished、Verified OK、Resetting Target。未全片擦除、未解除读保护或修改 Option Bytes。

ESP32：COM8，ESP32-S3 revision v0.2，MAC `b8:1f:3f:c4:6e:dc`，Flash 16 MB，内置 PSRAM 8 MB，USB Serial/JTAG。新应用 BIN SHA256 `77813854786732c4c74cef30e700dde083a8dc959a5a5ea7e9e839c6b9e996d1`。

ESP32 新一轮完整 16 MiB 读取在约 48% 时发生 USB 传输中断，没有得到完整新备份。随后读取本次全部写入范围所在的前 1 MiB：`backups/esp32s3_diagnostic_write_region_pre_motion_2026-10-11.bin`，1048576 字节，SHA256 `3dc21ac683490c77a2b6c513e2ef7536599f0ed4d6d560a32a40e82c631d6597`，verify_flash 返回 digest matched。原 2026-10-07 的完整 16 MiB 备份仍保留。

ESP32 已通过 PlatformIO 写入引导、分区表、初始 OTA 选择数据和应用，四组件均报告 Hash of data verified，最终 SUCCESS；擦除范围均在上述 1 MiB 备份内。未全片擦除或修改安全设置。

初次自动复位后没有日志或热点。随后尝试软件看门狗复位，并收到 v2 周期日志；用户明确告知期间按过 RST，不能把恢复单独归功于软件复位。用户要求今后遇到同类“已识别/已校验但无运行日志”的场景直接提醒按 RST。读取日志时曾遇到 COM8 短暂丢失，后续日志和热点恢复，仍需注意 USB 接触/供电状态。

## STM32 初次运行检查

SWD 运行态读取，未暂停 CPU：`motion_ready=1`、启动随机种子非零、token/flags/输出均为零。uwTick 从 `0x14584` 到 `0x1472f` 持续增加，未观察到复位。

TIM3 PSC=0、ARR=`0x63f`（1599），CCR1/CCR2=0。GPIOC MODER=`0x0000a000`，AFRL=`0x22000000`，PC6/PC7 为 AF2；GPIOB MODER=`0x55000281`，PB0 及 PB12–PB15 为输出；GPIOB ODR=0。确认固件配置与接线表一致，没有用仪器测 PWM 波形/电平。

## 网络

当前手机 USB `Apple Mobile Device Ethernet` 已出现，IP `172.20.10.2`，网关 `172.20.10.1`。绑定该地址的百度 HTTPS 请求返回 HTTP 200，可用于电脑 Wi-Fi 切换到 ESP32 AP 后的联网通道。

## 无动力运动协议验收

电脑已经连接 `ChassisDrive-Setup`，ESP32 报告 `CHASSIS_BRIDGE v2`、Flash=16777216、PSRAM 约 8 MiB、TCP=6001、motion=6002。

初次 v1 诊断请求超时。SWD 读取 STM32 接收队列证实 CM_STOP 和 CD_PING 请求完整且 CRC 有效，故不能把故障归为电脑未发送或 ESP32 到 STM32 方向不通。用户核对 STM32 PA2 → ESP32 GPIO18 回传线及共地后确认发现接错或松动，已修正；修正后回传恢复。

端到端诊断正式一轮 30/30 成功，最大往返 93 ms，回复标明运动实现存在；另外排查中的 2 次及后续读取状态也成功。正式实测详情保存在 Git 忽略目录 `logs/2026-10-11-motion/report.json` 及各 SWD 快照中。

所有非零指令均在用户明确确认动力断开的条件下执行。期间只读观察运行态 RAM/寄存器，没有暂停 CPU。

| 工况 | 实测结果 |
|---|---|
| 新会话 HELLO，未 ARM | STBY 低、左右 CCR=0、flags=0；存在待使能 token，不代表已使能 |
| 左轮 +100、右轮 0，持续刷新 | 6 个真实运动回复；CCR1=160、CCR2=0，PB12 与 PB0 高，ARR=1599 |
| 停止刷新，等待 400 ms 后检查 | STBY 低、两路 CCR=0、token=0、TIMED_OUT；旧 token 的 DRIVE 被拒绝 |
| 重新 HELLO/ARM，左轮 0、右轮 -150 | 5 个真实运动回复；CCR1=0、CCR2=240，PB15 与 PB0 高 |
| 显式 STOP | STBY 低、两路 CCR=0、token=0、flags=0 |
| 左右 +100，持续刷新 | 6 个真实运动回复；左右 CCR=160，方向/使能寄存器正确 |
| 直接关闭 TCP，跳过 PC STOP | 等待后 STBY 低、两路 CCR=0、token=0，观察到 TIMED_OUT 标志；本地命令租约发挥作用 |
| 新 TCP 连接、HELLO | 保持禁用，没有自动恢复上次运动 |
| 最终 STOP 与快照 | STBY 低、左右 CCR=0、token=0、flags=0 |

300 ms 是固件配置，C 控制核心已测试其边界；本轮真实超时检查在暂停刷新 400 ms 后再用 SWD 快照观察，未用示波器测量精确撤销输出延迟。GPIO ODR/CCR 配置正确不等于已测模块实际输入电平、输出波形或车轮物理停车。

最后阶段 COM8 暂时未枚举，但热点、真实 TCP/UART 往返及 STM32 状态均可工作。为排查回传曾临时准备 ESP32 串口接收计数，但该上传因 COM8 缺失未开始写入，后续回传线修正后撤掉该临时修改；当前实物运动代码对应提交 `2c091d0`。下一次需要再次刷写 ESP32 时，应重新枚举 USB，并按用户要求提醒使用可用的 RST。

## 逻辑线接好后的复查

用户随后确认 TB6612 逻辑线已接好，两板及 ST-Link USB 已接回，12 V 仍断开，暂时没有万用表。本次仅运行态 SWD 读取，没有暂停 CPU，没有发 ARM/DRIVE 或连接动力。

- ST-Link 可识别，报告 target voltage 约 3.27 V；该读数不能证明 TB6612 VCC 接线或模块端电压正确。
- GPIOB IDR=`0x0000001a`、ODR=0，PB0（STBY）输入采样和输出位均为零。
- GPIOC IDR=`0x00009f00`，PC6/PC7 输入采样均为零；TIM3 CCR1/CCR2 均为零。
- 控制 session/token/flags 及目标/输出均为零；`motion_ready` 低字节为 1，计时继续增加，程序在运行。
- 接线后最终快照保存在 Git 忽略目录 `logs/2026-10-11-motion/tb6612_logic_connected_default.txt`。
- 首次电脑仍在校园 Wi-Fi，未检测到 ESP32 串口或热点、未检测到手机 USB 共享网卡。按用户偏好提醒 ESP32 RST/EN，并请恢复手机 USB 共享。

用户确认按 ESP32 RST、恢复手机共享后，USB 共享网卡 `172.20.10.2` 出现，绕过当前进程的本机代理、绑定该地址的百度 HTTPS 返回 HTTP 200；ESP32 仍无串口或热点，同时 Windows 有 USB 描述符请求失败设备。用户随后将两板及 ST-Link 完整断电，ESP32 改接另一个电脑 USB 口后恢复供电，热点重新出现。

电脑已连接 `ChassisDrive-Setup`（BSSID `ba:1f:3f:c4:6e:dc`、IP `192.168.4.2`），切换后绑定手机 USB 的 HTTPS 仍返回 HTTP 200。ESP32 COM 仍未枚举，但无线桥接可用。本次未重新烧录，也未发送 ARM/DRIVE 或非零命令。

接线后实际诊断 10/10、最大往返 16 ms；真实 v2 HELLO 返回固件 ID `0x20261011`、flags=0、左右输出=0，显式 STOP 返回 token=0、flags=0、左右=0。随后 SWD 最终快照 GPIOB IDR=`0x1a`、ODR=0，GPIOC IDR=`0xdf00`（PC6/PC7 均低）、CCR1/CCR2=0、控制 flags/token=0，uwTick=108261。详情在 `logs/2026-10-11-motion/tb6612_logic_connected_report.json` 和 `tb6612_logic_connected_final.txt`。

以上确认 MCU 默认禁用配置及引脚采样，不能替代模块端实际供电、电平、输出或电机测量。

## 当前接续点

两板新固件已写入并做上述此前无动力验证，用户现已完成逻辑接线，接线后默认禁用、真实通信与 STOP 复查均通过。用户明确确认暂时没有降压模块，要求等到货后再推进；当前暂停动力测试，保持电池断开。

用户提供电池照片，标签为 WHEELTEC“12 V 安全锂电池组”，E326S、2600 mAh、充电电压 DC 12.6 V、最大持续放电 6 A、充电电流 ≤2 A。按标签充电目标，电压低于 TB6612 VM 工作上限 13.5 V；实际电池端电压和充电器输出仍未测。后续到货先核对降压模块并确认 5 V 输出，再给两板独立供电、断开通往电脑的 USB，核对电机接口后安排架空逐轮测试。

芯片手册第 2 页确认 STBY 内置 200 kΩ 下拉，额外约 10 kΩ 为建议，实际模块不得固定上拉使能。编码器、实际轮向、温升、失联物理停稳及负载能力仍未验收。
