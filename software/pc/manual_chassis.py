"""Testable manual-control safety state and single-worker TCP transport.

This is an operator aid, not a hardware emergency stop. The STM32 lease and
board-level motor disable still have to be implemented and verified.
"""

from dataclasses import dataclass
from enum import Enum
import math
import threading
import time

from motion_link import MotionLinkClient
from motion_protocol import MotionCommand, sequence_is_newer
from rover_telemetry import RoverTelemetry, TelemetryStatus


class ControlState(Enum):
    DISCONNECTED = "未连接"
    SYNCHRONIZING = "等待反馈 / 零速确认"
    DISARMED = "已连接，未允许运动"
    ARMED = "已允许运动"
    FAULT = "故障锁定，需要重新连接"


@dataclass(frozen=True)
class PendingCommand:
    command: MotionCommand
    sent_s: float


@dataclass(frozen=True)
class ManualStatus:
    state: ControlState
    reason: str
    can_arm: bool
    requested_speeds: tuple[int, int]
    sent_speeds: tuple[int, int]
    measured_speeds: tuple[float, float] | None
    telemetry: RoverTelemetry | None
    telemetry_age_s: float | None
    pending_sequence: int | None


class ManualChassisController:
    """Caller owns locking and supplies monotonic seconds (unit-test friendly)."""

    def __init__(
        self, speed_mm_s: int = 80, *, wheel_diameter_mm: float | None = None,
        counts_per_rev: int = 1560,
    ):
        if wheel_diameter_mm is not None and (
            not math.isfinite(wheel_diameter_mm) or wheel_diameter_mm <= 0
        ):
            raise ValueError("measured wheel diameter must be positive and finite")
        if type(counts_per_rev) is not int or counts_per_rev <= 0:
            raise ValueError("counts/revolution must be a positive integer")
        self.mm_per_count = (
            math.pi * wheel_diameter_mm / counts_per_rev if wheel_diameter_mm is not None else None
        )
        self.speed_mm_s = 80
        self.set_speed(speed_mm_s)
        self.send_period_s = 0.05
        self.lease_ms = 300
        self.telemetry_timeout_s = 0.30
        self.ack_timeout_s = 0.20
        self.first_telemetry_timeout_s = 0.80
        self.input_timeout_s = 0.25
        self.state = ControlState.DISCONNECTED
        self.reason = "请先连接；连接后仍需显式允许运动"
        self.keys: set[str] = set()
        self._held_keys: set[str] = set()
        self._blocked_keys: set[str] = set()
        self.frame: RoverTelemetry | None = None
        self.received_s: float | None = None
        self.pending: PendingCommand | None = None
        self.measured_speeds: tuple[float, float] | None = None
        self.sent_speeds = (0, 0)
        self._next_sequence: int | None = None
        self._last_ack: int | None = None
        self._zero_confirmed = False
        self._need_zero = False
        self._next_send_s = 0.0
        self._started_s = 0.0
        self._input_alive_s = 0.0

    def set_speed(self, speed_mm_s: int) -> None:
        if type(speed_mm_s) is not int or not 1 <= speed_mm_s <= 300:
            raise ValueError("manual speed must be an integer in 1..300 mm/s")
        self.speed_mm_s = speed_mm_s

    def begin_connection(self, now_s: float) -> None:
        self.state = ControlState.SYNCHRONIZING
        self.reason = "等待新鲜遥测，并同步命令序号"
        self.keys.clear()
        self.frame = None
        self.received_s = None
        self.pending = None
        self.measured_speeds = None
        self.sent_speeds = (0, 0)
        self._next_sequence = None
        self._last_ack = None
        self._zero_confirmed = False
        self._need_zero = True
        self._next_send_s = now_s
        self._started_s = now_s
        self._input_alive_s = now_s

    def input_alive(self, now_s: float) -> None:
        """GUI must call regularly; a hung UI must not maintain held-key motion."""
        self._input_alive_s = now_s

    def fail(self, reason: str) -> None:
        self.keys.clear()
        self._zero_confirmed = False
        self._need_zero = True
        self.state = ControlState.FAULT
        self.reason = reason

    def disconnected(self) -> None:
        self.keys.clear()
        self._zero_confirmed = False
        if self.state != ControlState.FAULT:
            self.state = ControlState.DISCONNECTED
            self.reason = "已断开；再次连接后需要重新允许运动"

    def disarm(self, reason: str = "操作者停止") -> None:
        self.keys.clear()
        self._need_zero = True
        self._zero_confirmed = False
        if self.state in (ControlState.ARMED, ControlState.DISARMED):
            self.state = ControlState.DISARMED
            self.reason = reason + "；等待零速指令确认"

    def _check_health(self, now_s: float) -> None:
        if self.state in (ControlState.DISCONNECTED, ControlState.FAULT):
            return
        if self.received_s is None:
            if now_s - self._started_s >= self.first_telemetry_timeout_s:
                self.fail("未收到 STM32 遥测，禁止运动")
        elif now_s - self.received_s >= self.telemetry_timeout_s:
            self.fail("遥测超时，禁止运动")
        if self.pending is not None and now_s - self.pending.sent_s >= self.ack_timeout_s:
            self.fail("指令未被 STM32 确认，禁止运动")
        if self.state == ControlState.ARMED and now_s - self._input_alive_s >= self.input_timeout_s:
            self.fail("控制界面无响应，禁止运动")

    def arm(self, now_s: float) -> bool:
        self._check_health(now_s)
        if self.state != ControlState.DISARMED or not self._zero_confirmed:
            return False
        self.keys.clear()  # Pressing W before arming never starts motion.
        self._blocked_keys = self._held_keys.copy()  # Ignore auto-repeat until a real release.
        self._input_alive_s = now_s
        self.state = ControlState.ARMED
        self.reason = "按住 W/S/A/D 或方向按钮运行；松开停止"
        return True

    def press(self, key: str) -> None:
        key = key.lower()
        if key not in ("w", "a", "s", "d"):
            return
        self._held_keys.add(key)
        if self.state == ControlState.ARMED and key not in self._blocked_keys:
            self.keys.add(key)

    def release(self, key: str) -> None:
        self.keys.discard(key.lower())
        self._held_keys.discard(key.lower())
        self._blocked_keys.discard(key.lower())
        self._need_zero = True

    def desired_speeds(self) -> tuple[int, int]:
        if self.state != ControlState.ARMED:
            return (0, 0)
        if {"w", "s"} <= self.keys or {"a", "d"} <= self.keys:
            return (0, 0)
        forward = int("w" in self.keys) - int("s" in self.keys)
        left_turn = int("a" in self.keys) - int("d" in self.keys)
        left, right = forward - left_turn, forward + left_turn
        scale = max(1, abs(left), abs(right))
        return (round(left * self.speed_mm_s / scale), round(right * self.speed_mm_s / scale))

    def observe(self, frame: RoverTelemetry, now_s: float) -> None:
        # A late acknowledgement must not erase an already-expired deadline.
        self._check_health(now_s)
        if self.state in (ControlState.DISCONNECTED, ControlState.FAULT):
            return
        previous = self.frame
        if previous is not None:
            delta_ms = (frame.stm32_ticks_ms - previous.stm32_ticks_ms) & 0xFFFFFFFF
            if delta_ms >= 0x80000000:
                self.fail("STM32 复位或时钟回退，需要重新连接")
                return
            if not sequence_is_newer(frame.sequence, previous.sequence):
                return  # Duplicates do not refresh freshness or acknowledgement.
        else:
            delta_ms = 0
        required = TelemetryStatus.LEFT_ENCODER_VALID | TelemetryStatus.RIGHT_ENCODER_VALID
        if frame.status & TelemetryStatus.EMERGENCY_STOP:
            self.fail("STM32 报告急停，禁止运动")
            return
        if frame.status & required != required:
            self.fail("左右编码器反馈未就绪，禁止运动")
            return
        self.measured_speeds = None
        if previous is not None and self.mm_per_count is not None and 0 < delta_ms <= 300:
            counts = (
                (frame.left_encoder_total - previous.left_encoder_total + 2**31) % 2**32 - 2**31,
                (frame.right_encoder_total - previous.right_encoder_total + 2**31) % 2**32 - 2**31,
            )
            self.measured_speeds = tuple(value * self.mm_per_count * 1000 / delta_ms for value in counts)
        self.frame = frame
        self.received_s = now_s
        has_ack = bool(frame.status & TelemetryStatus.LAST_COMMAND_VALID)
        ack = frame.last_command_sequence
        if self._next_sequence is None:
            self._last_ack = ack if has_ack else None
            self._next_sequence = ((ack + 1) & 0xFFFFFFFF) if has_ack else 1
            return
        if not has_ack:
            if self._last_ack is not None:
                self.fail("STM32 命令状态重置，需要重新连接")
            return
        if self.pending is not None:
            sent = self.pending.command
            if ack == sent.sequence:
                self._last_ack = ack
                self.pending = None
                if sent.left_mm_s == 0 and sent.right_mm_s == 0:
                    self._zero_confirmed = True
                    if self.state == ControlState.SYNCHRONIZING:
                        self.state = ControlState.DISARMED
                    if self.state == ControlState.DISARMED:
                        self.reason = "零速指令已被接收；可显式允许运动（不代表电机已停稳）"
            elif sequence_is_newer(ack, sent.sequence):
                self.fail("出现非本控制端的命令序号，禁止运动")
        elif self._last_ack is not None and ack != self._last_ack:
            self.fail("出现非本控制端的命令序号，禁止运动")

    def poll(self, now_s: float) -> MotionCommand | None:
        self._check_health(now_s)
        if self.state in (ControlState.DISCONNECTED, ControlState.FAULT) or self._next_sequence is None:
            return None
        left, right = self.desired_speeds()
        if self.pending is not None:
            sent = self.pending.command
            stopping = (left, right) == (0, 0) and (sent.left_mm_s, sent.right_mm_s) != (0, 0)
            if not (stopping and self._need_zero):
                return None  # Never queue a stream of unacknowledged motion frames.
        if now_s < self._next_send_s and not (self._need_zero and (left, right) == (0, 0)):
            return None
        command = MotionCommand(self._next_sequence, self.lease_ms, left, right)
        self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF
        self.pending = PendingCommand(command, now_s)
        self.sent_speeds = (left, right)
        if (left, right) != (0, 0):
            self._zero_confirmed = False
        self._next_send_s = now_s + self.send_period_s
        self._need_zero = False
        return command

    def final_zero(self) -> MotionCommand | None:
        """Best effort before transport close; receipt is NOT guaranteed."""
        self.keys.clear()
        if self._next_sequence is None:
            return None
        command = MotionCommand(self._next_sequence, self.lease_ms, 0, 0)
        self._next_sequence = (self._next_sequence + 1) & 0xFFFFFFFF
        self.sent_speeds = (0, 0)
        return command

    def status(self, now_s: float) -> ManualStatus:
        return ManualStatus(
            self.state, self.reason,
            self.state == ControlState.DISARMED and self._zero_confirmed
            and self.received_s is not None and now_s - self.received_s < self.telemetry_timeout_s,
            self.desired_speeds(), self.sent_speeds, self.measured_speeds, self.frame,
            None if self.received_s is None else now_s - self.received_s,
            None if self.pending is None else self.pending.command.sequence,
        )


class ManualMotionWorker:
    """All socket access happens here, never on the GUI thread. No auto-reconnect."""

    def __init__(self, controller: ManualChassisController, host: str, port: int):
        self.controller = controller
        self.lock = threading.RLock()
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            raise RuntimeError("control worker is already running")
        self._stop.clear()
        with self.lock:
            self.controller.begin_connection(time.monotonic())
        self._thread = threading.Thread(target=self._run, name="manual-motion", daemon=True)
        self._thread.start()

    def request_stop(self) -> None:
        with self.lock:
            self.controller.disarm("断开连接")
        self._stop.set()

    def close(self) -> None:
        self.request_stop()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
        if self.running:
            raise RuntimeError("motion worker did not stop")

    def _run(self) -> None:
        link = MotionLinkClient(self.host, self.port, connect_timeout_s=0.5)
        try:
            link.connect()
            while not self._stop.is_set():
                # Bounded drain prevents a busy/malicious peer starving the watchdog.
                for _ in range(64):
                    frame = link.receive_telemetry(timeout_s=0.0)
                    if frame is None:
                        break
                    with self.lock:
                        self.controller.observe(frame, time.monotonic())
                        if self.controller.state == ControlState.FAULT:
                            break
                with self.lock:
                    command = self.controller.poll(time.monotonic())
                    if self.controller.state == ControlState.FAULT:
                        break
                    if command is not None:
                        # Lock covers decision+bounded send so stop cannot race an old target.
                        link.send_command(command, timeout_s=0.05)
                self._stop.wait(0.005)
        except Exception as exc:
            with self.lock:
                if not self._stop.is_set():
                    self.controller.fail("通信失败：" + str(exc))
        finally:
            with self.lock:
                zero = self.controller.final_zero()
            if link.connected and zero is not None:
                try:
                    link.send_command(zero, timeout_s=0.05)
                except OSError:
                    pass
            link.close()
            with self.lock:
                self.controller.disconnected()
