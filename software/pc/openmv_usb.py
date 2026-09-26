"""Bounded OpenMV 3.x/4.x USBDBG client (not the ESP32 RSV1 protocol).

Protocol reference: https://github.com/openmv/openmv/blob/v4.5.9/tools/pyopenmv.py
No flash, filesystem, bootloader, or reset operations are implemented here.
The caller owns a pyserial-compatible port with finite read/write timeouts.
"""

from dataclasses import dataclass
import math
import struct
import time

import cv2
import numpy as np


COMMAND = struct.Struct("<BBI")
WORDS = struct.Struct("<III")
MAX_FRAME_BYTES = 4 * 1024 * 1024


@dataclass(frozen=True)
class USBFrame:
    width: int
    height: int
    format: str
    payload: bytes
    image: np.ndarray


def decode_frame(width: int, height: int, size: int, payload: bytes) -> USBFrame:
    expected = frame_bytes(width, height, size)
    if len(payload) != expected:
        raise ValueError("Incomplete USB image")
    if size == 1:
        gray = np.frombuffer(payload, np.uint8).reshape(height, width)
        image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        fmt = "GRAY8"
    elif size == 2:
        pixels = np.frombuffer(payload, dtype="<u2").reshape(height, width)
        # Convert RGB565 to OpenCV BGR without native-endian assumptions.
        image = np.stack(((pixels & 31) * 255 // 31,
                          ((pixels >> 5) & 63) * 255 // 63,
                          ((pixels >> 11) & 31) * 255 // 31), axis=-1).astype(np.uint8)
        fmt = "RGB565"
    else:
        image = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (height, width):
            raise ValueError("Invalid JPEG or dimensions differ from USB header")
        fmt = "JPEG"
    return USBFrame(width, height, fmt, payload, image)


def frame_bytes(width: int, height: int, size: int) -> int:
    if not (1 <= width <= 4096 and 1 <= height <= 4096 and size >= 1):
        raise ValueError("Invalid USB frame header")
    count = width * height * size if size <= 2 else size
    if count > MAX_FRAME_BYTES:
        raise ValueError("USB frame exceeds allocation limit")
    return count


class OpenMVUSB:
    def __init__(self, port, *, timeout: float = 3.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be positive")
        self.port = port
        self.timeout = timeout

    def _write(self, data: bytes) -> None:
        offset = 0
        deadline = time.monotonic() + self.timeout
        while offset < len(data):
            if time.monotonic() >= deadline:
                raise TimeoutError("USB write deadline exceeded")
            count = self.port.write(data[offset:])
            if not count:
                raise TimeoutError("USB write stalled")
            offset += count

    def _read(self, count: int) -> bytes:
        result = bytearray()
        deadline = time.monotonic() + self.timeout
        while len(result) < count:
            if time.monotonic() >= deadline:
                raise TimeoutError(f"USB reply incomplete: {len(result)}/{count} bytes")
            result.extend(self.port.read(count - len(result)))
        return bytes(result)

    def _command(self, command: int, count: int) -> None:
        self._write(COMMAND.pack(0x30, command, count))

    def info(self) -> dict:
        self._command(0x80, 12)
        version = WORDS.unpack(self._read(12))
        if version[0] not in (3, 4):
            raise RuntimeError(f"Unsupported USBDBG firmware {version}; no script started")
        self._command(0x83, 64)
        board = self._read(64).split(b"\0", 1)[0].decode("utf-8", errors="replace")
        return {"firmware": ".".join(map(str, version)), "board": board}

    def stop_script(self) -> None:
        self._command(0x06, 0)

    def execute(self, script: str) -> None:
        data = script.encode("utf-8")
        if not data or len(data) > 64 * 1024:
            raise ValueError("Preview script is empty or too large")
        self._command(0x05, len(data))
        self._write(data)

    def enable_framebuffer(self, enabled: bool) -> None:
        self._command(0x0D, 4)
        self._write(struct.pack("<I", int(enabled)))

    def read_frame(self) -> USBFrame | None:
        self._command(0x81, 12)
        width, height, size = WORDS.unpack(self._read(12))
        if width == 0:
            return None
        count = frame_bytes(width, height, size)
        self._command(0x82, count)
        return decode_frame(width, height, size, self._read(count))

    def read_log(self) -> str:
        self._command(0x8E, 4)
        count = struct.unpack("<I", self._read(4))[0]
        if count > 64 * 1024:
            raise ValueError("Invalid USB log length")
        if not count:
            return ""
        self._command(0x8F, count)
        return self._read(count).decode("utf-8", errors="replace")
