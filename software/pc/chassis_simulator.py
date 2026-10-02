"""Loopback-only RCM1/RST1 peer. Ideal wheels, NOT STM32/driver validation."""

import argparse
from dataclasses import dataclass
import math
import socket
import threading
import time

from motion_protocol import MotionCommand, MotionCommandDecoder, sequence_is_newer
from rover_telemetry import (
    INVALID_RANGE_MM, RoverTelemetry, TelemetryStatus, encode_telemetry,
)


def signed32(value: int) -> int:
    return (value + 2**31) % 2**32 - 2**31


@dataclass(frozen=True)
class SimulatedWheels:
    left_mm_s: int
    right_mm_s: int
    left_counts: int
    right_counts: int
    last_command_sequence: int | None


class ChassisSimulation:
    """Deterministic lease/sequence model with synthetic encoder geometry.

    A dropped TCP connection does not magically notify the simulated MCU:
    the last target expires on its lease, as on a transparent UART bridge.
    """

    def __init__(self, wheel_diameter_mm: float = 75.0, counts_per_rev: int = 1560):
        if not math.isfinite(wheel_diameter_mm) or wheel_diameter_mm <= 0:
            raise ValueError("wheel diameter must be positive and finite")
        if type(counts_per_rev) is not int or counts_per_rev <= 0:
            raise ValueError("counts/revolution must be a positive integer")
        self.counts_per_mm = counts_per_rev / (math.pi * wheel_diameter_mm)
        self.now_s = 0.0
        self.last_sequence: int | None = None
        self.command: MotionCommand | None = None
        self.received_s = 0.0
        self.left_count = 0.0
        self.right_count = 0.0
        self.telemetry_sequence = 0
        self.emergency_stop = False
        self.encoders_valid = True

    def advance(self, now_s: float) -> None:
        if not math.isfinite(now_s) or now_s < self.now_s:
            raise ValueError("simulation clock must be finite and monotonic")
        if self.command is not None:
            expires_s = self.received_s + self.command.lease_ms / 1000.0
            active_dt = max(0.0, min(now_s, expires_s) - self.now_s)
            if not self.emergency_stop and self.encoders_valid:
                self.left_count += self.command.left_mm_s * active_dt * self.counts_per_mm
                self.right_count += self.command.right_mm_s * active_dt * self.counts_per_mm
            if now_s >= expires_s or self.emergency_stop or not self.encoders_valid:
                self.command = None
        self.now_s = now_s

    def accept(self, command: MotionCommand, now_s: float) -> bool:
        self.advance(now_s)
        if self.last_sequence is not None and not sequence_is_newer(command.sequence, self.last_sequence):
            return False
        self.command = command
        self.last_sequence = command.sequence
        self.received_s = now_s
        self.advance(now_s)
        return True

    def snapshot(self) -> SimulatedWheels:
        command = self.command
        return SimulatedWheels(
            command.left_mm_s if command else 0,
            command.right_mm_s if command else 0,
            signed32(round(self.left_count)), signed32(round(self.right_count)),
            self.last_sequence,
        )

    def telemetry(self) -> RoverTelemetry:
        status = TelemetryStatus(0)
        if self.encoders_valid:
            status |= TelemetryStatus.LEFT_ENCODER_VALID | TelemetryStatus.RIGHT_ENCODER_VALID
        if self.emergency_stop:
            status |= TelemetryStatus.EMERGENCY_STOP
        if self.last_sequence is not None:
            status |= TelemetryStatus.LAST_COMMAND_VALID
        wheels = self.snapshot()
        result = RoverTelemetry(
            self.telemetry_sequence, int(self.now_s * 1000) & 0xFFFFFFFF,
            self.last_sequence if self.last_sequence is not None else 0,
            wheels.left_counts, wheels.right_counts, 0,
            INVALID_RANGE_MM, INVALID_RANGE_MM, 0, status,
        )
        self.telemetry_sequence = (self.telemetry_sequence + 1) & 0xFFFFFFFF
        return result


class ChassisSimulatorServer:
    """One-client localhost TCP server; start/close manage the background thread."""

    def __init__(
        self, port: int = 0, *, wheel_diameter_mm: float = 75.0,
        drop_commands: bool = False, mute_telemetry_after_s: float | None = None,
        emergency_stop_after_s: float | None = None,
    ):
        if type(port) is not int or not 0 <= port <= 65535:
            raise ValueError("port must be in 0..65535")
        for value in (mute_telemetry_after_s, emergency_stop_after_s):
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError("fault time must be nonnegative and finite")
        self.model = ChassisSimulation(wheel_diameter_mm)
        self.drop_commands = drop_commands
        self.mute_telemetry_after_s = mute_telemetry_after_s
        self.emergency_stop_after_s = emergency_stop_after_s
        self.mute_telemetry = threading.Event()
        self._model_lock = threading.Lock()
        self._client_lock = threading.Lock()
        self._client: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self._listener.bind(("127.0.0.1", port))
            self._listener.listen(1)
            self._listener.settimeout(0.02)
        except BaseException:
            self._listener.close()
            raise
        self.host, self.port = self._listener.getsockname()
        self._boot_s = time.monotonic()
        self.error: str | None = None

    def _advance_locked(self) -> float:
        now_s = time.monotonic() - self._boot_s
        # Integrate the old state up to the exact injected emergency time.
        if self.emergency_stop_after_s is not None and now_s >= self.emergency_stop_after_s:
            if self.model.now_s <= self.emergency_stop_after_s:
                self.model.advance(self.emergency_stop_after_s)
            self.model.emergency_stop = True
        self.model.advance(now_s)
        return now_s

    def snapshot(self) -> SimulatedWheels:
        with self._model_lock:
            self._advance_locked()
            return self.model.snapshot()

    def start(self):
        if self._thread is not None:
            raise RuntimeError("simulator already started")
        self._thread = threading.Thread(target=self._run, name="chassis-simulator", daemon=True)
        self._thread.start()
        return self

    def disconnect_client(self) -> None:
        with self._client_lock:
            if self._client is not None:
                try:
                    self._client.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

    def close(self) -> None:
        self._stop.set()
        self.disconnect_client()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._listener.close()
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("simulator thread did not stop")

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                self.snapshot()  # Watchdog continues even without a TCP client.
                try:
                    client, _ = self._listener.accept()
                except socket.timeout:
                    continue
                with client:
                    with self._client_lock:
                        self._client = client
                    try:
                        self._serve(client)
                    except (OSError, ConnectionError):
                        pass  # A normal client disconnect is not a server failure.
                    finally:
                        with self._client_lock:
                            self._client = None
        except Exception as exc:
            self.error = str(exc)

    def _serve(self, client: socket.socket) -> None:
        client.settimeout(0.01)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        decoder = MotionCommandDecoder()
        next_telemetry_s = 0.0
        while not self._stop.is_set():
            with self._model_lock:
                now_s = self._advance_locked()
                muted = self.mute_telemetry.is_set() or (
                    self.mute_telemetry_after_s is not None and now_s >= self.mute_telemetry_after_s
                )
                wire = None
                if now_s >= next_telemetry_s and not muted:
                    wire = encode_telemetry(self.model.telemetry())
                    next_telemetry_s = now_s + 0.05
            if wire is not None:
                client.sendall(wire)
            try:
                chunk = client.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return
            with self._model_lock:
                now_s = self._advance_locked()
                for command in decoder.feed(chunk):
                    if not self.drop_commands:
                        self.model.accept(command, now_s)

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Local chassis protocol simulator; no hardware access")
    parser.add_argument("--port", type=int, default=5501)
    parser.add_argument("--wheel-diameter-mm", type=float, default=75.0, help="Synthetic geometry, not your wheel measurement")
    parser.add_argument("--drop-commands", action="store_true")
    parser.add_argument("--mute-telemetry-after", type=float)
    parser.add_argument("--emergency-stop-after", type=float)
    args = parser.parse_args()
    try:
        with ChassisSimulatorServer(
            args.port, wheel_diameter_mm=args.wheel_diameter_mm,
            drop_commands=args.drop_commands,
            mute_telemetry_after_s=args.mute_telemetry_after,
            emergency_stop_after_s=args.emergency_stop_after,
        ) as server:
            print(f"SIMULATION ONLY: {server.host}:{server.port}; Ctrl+C to stop", flush=True)
            while server.error is None:
                time.sleep(0.2)
            raise RuntimeError(server.error)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
