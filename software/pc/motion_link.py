"""Blocking TCP client for the ESP32 motion bridge on port 5001."""

from collections import deque
import math
import socket
import threading

from motion_protocol import MotionCommand, encode_motion_command
from rover_telemetry import RoverTelemetry, TelemetryDecoder


class MotionLinkClient:
    """Send leased commands and receive decoded STM32 telemetry.

    Use on the command/control loop or provide a short receive timeout. The
    STM32 lease watchdog remains the safety boundary if the PC or Wi-Fi link
    stops sending; TCP delivery alone is not treated as proof of actuation.
    """

    def __init__(
        self,
        host: str = "192.168.4.1",
        port: int = 5001,
        *,
        connect_timeout_s: float = 5.0,
        receive_buffer_size: int = 4096,
    ) -> None:
        if not host:
            raise ValueError("host cannot be empty")
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError("port must be in 1..65535")
        if not math.isfinite(connect_timeout_s) or connect_timeout_s <= 0:
            raise ValueError("connect_timeout_s must be positive")
        if type(receive_buffer_size) is not int or receive_buffer_size < 38:
            raise ValueError("receive_buffer_size must be at least one telemetry frame")
        self.host = host
        self.port = port
        self.connect_timeout_s = connect_timeout_s
        self.receive_buffer_size = receive_buffer_size
        self._socket: socket.socket | None = None
        self._decoder = TelemetryDecoder()
        self._pending: deque[RoverTelemetry] = deque()
        self._send_lock = threading.Lock()

    @property
    def connected(self) -> bool:
        return self._socket is not None

    @property
    def decoder(self) -> TelemetryDecoder:
        """Expose decoder counters for diagnostics without exposing its buffer."""
        return self._decoder

    def connect(self) -> None:
        if self._socket is not None:
            return
        connection = socket.create_connection(
            (self.host, self.port),
            timeout=self.connect_timeout_s,
        )
        connection.settimeout(None)
        self._decoder = TelemetryDecoder()
        self._pending.clear()
        self._socket = connection

    def send_command(self, command: MotionCommand) -> None:
        frame = encode_motion_command(command)
        connection = self._require_socket()
        try:
            with self._send_lock:
                connection.sendall(frame)
        except OSError:
            self.close()
            raise

    def receive_telemetry(self, timeout_s: float = 0.0) -> RoverTelemetry | None:
        """Return the next telemetry frame, or ``None`` on receive timeout."""
        if not math.isfinite(timeout_s) or timeout_s < 0:
            raise ValueError("timeout_s cannot be negative")
        if self._pending:
            return self._pending.popleft()
        connection = self._require_socket()
        previous_timeout = connection.gettimeout()
        try:
            connection.settimeout(timeout_s)
            while True:
                try:
                    chunk = connection.recv(self.receive_buffer_size)
                except (socket.timeout, BlockingIOError):
                    return None
                if not chunk:
                    self.close()
                    raise ConnectionError("ESP32 motion connection closed")
                self._pending.extend(self._decoder.feed(chunk))
                if self._pending:
                    return self._pending.popleft()
        except OSError:
            self.close()
            raise
        finally:
            if self._socket is connection:
                connection.settimeout(previous_timeout)

    def close(self) -> None:
        connection, self._socket = self._socket, None
        self._pending.clear()
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise ConnectionError("motion link is not connected; call connect() first")
        return self._socket

    def __enter__(self) -> "MotionLinkClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
