"""Phase-2 zero-only TCP diagnostic. Default peer is an embedded simulator."""

import argparse
from dataclasses import asdict, dataclass
import ipaddress
import json
import math
import time

from chassis_simulator import ChassisSimulation, ChassisSimulatorServer
from motion_link import MotionLinkClient
from motion_protocol import ActuatorCommand, MotionCommand, sequence_is_newer
from rover_telemetry import RoverTelemetry, TelemetryStatus


class CommunicationOnlySimulation(ChassisSimulation):
    """Software-only stage-2 profile, not execution of STM32 registers/HAL."""

    def __init__(self):
        super().__init__()
        self.encoders_valid = False
        self.emergency_stop = True

    def accept(self, command: MotionCommand, now_s: float) -> bool:
        if command.left_mm_s or command.right_mm_s or command.actuator != ActuatorCommand.HOLD:
            return False
        return super().accept(command, now_s)


def make_demo_server(**kwargs):
    server = ChassisSimulatorServer(**kwargs)
    server.model = CommunicationOnlySimulation()  # Replace before starting the thread.
    return server


@dataclass(frozen=True)
class ZeroProbeReport:
    received_frames: int
    acknowledged_zero_frames: int
    sent_zero_frames: int
    last_confirmed_sequence: int
    max_ack_round_trip_ms: float
    elapsed_s: float
    software_interlock_reported: bool = True
    hardware_verified: bool = False


class ZeroProbeSession:
    def __init__(self, started_s: float):
        self.started_s = started_s
        self.frame: RoverTelemetry | None = None
        self.received_s: float | None = None
        self.next_sequence: int | None = None
        self.last_ack: int | None = None
        self.pending: tuple[int, float] | None = None
        self.next_send_s = started_s
        self.received_frames = 0
        self.sent_zero_frames = 0
        self.acknowledged_zero_frames = 0
        self.max_ack_round_trip_ms = 0.0

    def check_timeouts(self, now_s):
        if self.received_s is None:
            if now_s - self.started_s >= 0.8:
                raise TimeoutError("no STM32 telemetry received")
        elif now_s - self.received_s >= 0.3:
            raise TimeoutError("STM32 telemetry expired")
        if self.pending is not None and now_s - self.pending[1] >= 0.2:
            raise TimeoutError("zero command not acknowledged by STM32")

    def observe(self, frame: RoverTelemetry, now_s):
        self.check_timeouts(now_s)  # Late replies cannot erase expired deadlines.
        previous = self.frame
        if previous is not None:
            delta = (frame.stm32_ticks_ms - previous.stm32_ticks_ms) & 0xFFFFFFFF
            if delta >= 0x80000000:
                raise RuntimeError("STM32 reset/clock regression; rerun probe to resynchronize")
            if not sequence_is_newer(frame.sequence, previous.sequence):
                return
        allowed = TelemetryStatus.EMERGENCY_STOP | TelemetryStatus.LAST_COMMAND_VALID
        if not frame.status & TelemetryStatus.EMERGENCY_STOP or frame.status & ~allowed:
            raise RuntimeError("peer does not report the communication-only software interlock profile")
        self.frame = frame
        self.received_s = now_s
        self.received_frames += 1
        has_ack = bool(frame.status & TelemetryStatus.LAST_COMMAND_VALID)
        ack = frame.last_command_sequence
        if self.sent_zero_frames == 0:
            self.last_ack = ack if has_ack else None
            self.next_sequence = ((ack + 1) & 0xFFFFFFFF) if has_ack else 1
            return
        if not has_ack:
            if self.last_ack is not None:
                raise RuntimeError("STM32 command state reset")
            return
        if self.pending is not None:
            sequence, sent_s = self.pending
            if ack == sequence:
                self.max_ack_round_trip_ms = max(self.max_ack_round_trip_ms, (now_s - sent_s) * 1000)
                self.acknowledged_zero_frames += 1
                self.last_ack = ack
                self.pending = None
            elif sequence_is_newer(ack, sequence):
                raise RuntimeError("another command producer or unsynchronized sequence detected")
        elif self.last_ack is not None and ack != self.last_ack:
            raise RuntimeError("another command producer detected")

    def poll(self, now_s, *, send_enabled=True):
        self.check_timeouts(now_s)
        if not send_enabled or self.next_sequence is None or self.pending is not None or now_s < self.next_send_s:
            return None
        command = MotionCommand(self.next_sequence, 300, 0, 0, ActuatorCommand.HOLD)
        self.next_sequence = (self.next_sequence + 1) & 0xFFFFFFFF
        self.pending = (command.sequence, now_s)
        self.next_send_s = now_s + 0.05
        self.sent_zero_frames += 1
        return command


def run_zero_probe(link: MotionLinkClient, duration_s: float = 3.0) -> ZeroProbeReport:
    if not math.isfinite(duration_s) or not 0.2 <= duration_s <= 60:
        raise ValueError("duration must be between 0.2 and 60 seconds")
    session = None
    try:
        link.connect()
        session = ZeroProbeSession(time.monotonic())
        ends_s = session.started_s + duration_s
        while True:
            for _ in range(32):
                frame = link.receive_telemetry(timeout_s=0)
                if frame is None:
                    break
                session.observe(frame, time.monotonic())
            now_s = time.monotonic()
            command = session.poll(now_s, send_enabled=now_s < ends_s)
            if command is not None:
                link.send_command(command, timeout_s=0.05)
            if now_s >= ends_s and session.pending is None:
                break
            time.sleep(0.005)
        if session.acknowledged_zero_frames == 0 or session.last_ack is None:
            raise RuntimeError("no confirmed zero command; probe failed")
        return ZeroProbeReport(
            session.received_frames, session.acknowledged_zero_frames,
            session.sent_zero_frames, session.last_ack, session.max_ack_round_trip_ms,
            time.monotonic() - session.started_s,
        )
    finally:
        if session is not None and session.next_sequence is not None and link.connected:
            try:
                # Unconfirmed best effort, deliberately not counted as an acknowledged frame.
                link.send_command(MotionCommand(session.next_sequence, 300, 0, 0), timeout_s=0.05)
            except OSError:
                pass
        link.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Zero-only stage-2 diagnostic; defaults to local simulation")
    peer = parser.add_mutually_exclusive_group()
    peer.add_argument("--demo", action="store_true")
    peer.add_argument("--host", help="Numeric IPv4 address; ESP32 motion port is 5001")
    parser.add_argument("--port", type=int)
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.host is None:
        if args.port is not None or args.allow_hardware:
            parser.error("--port / --allow-hardware require --host")
    else:
        try:
            address = ipaddress.IPv4Address(args.host)
        except ipaddress.AddressValueError:
            parser.error("--host must be numeric IPv4")
        if not address.is_loopback and not args.allow_hardware:
            parser.error("non-loopback targets require --allow-hardware and disconnected motor power")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("--port must be in 1..65535")
    if not math.isfinite(args.duration) or not 0.2 <= args.duration <= 60:
        parser.error("--duration must be 0.2..60 seconds")
    return args


def main():
    args = parse_args()
    server = None
    try:
        if args.host is None:
            server = make_demo_server().start()
            host, port = server.host, server.port
        else:
            host, port = args.host, args.port or 5001
        report = run_zero_probe(MotionLinkClient(host, port, connect_timeout_s=0.5), args.duration)
        print(json.dumps({"mode": "simulation" if server else "external-peer",
                          "endpoint": f"{host}:{port}", **asdict(report)}, indent=2))
    except KeyboardInterrupt:
        raise SystemExit("Zero-only probe interrupted; physical stop is not guaranteed") from None
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"Zero-only probe FAILED: {exc}") from None
    finally:
        if server is not None:
            server.close()


if __name__ == "__main__":
    main()
