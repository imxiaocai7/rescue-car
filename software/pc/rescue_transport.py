"""OpenMV -> ESP32 -> PC image framing, version 1.

Wire format (little endian):
    magic[4], version:u8, kind:u8, seq:u32, camera_ticks_ms:u32,
    width:u16, height:u16, jpeg_size:u32, jpeg_crc32:u32, jpeg bytes.

The ESP32 should forward the complete frame unchanged over TCP.  This module
does not send motion commands; it is safe to test before the motor driver is
selected.
"""

from dataclasses import dataclass
import struct
import zlib


MAGIC = b"RSV1"
VERSION = 1
KIND_JPEG = 1
HEADER = struct.Struct("<4sBBIIHHII")
ACK_MAGIC = b"ACK1"
FRAME_ACK = struct.Struct("<4sI")
MAX_JPEG_SIZE = 120_000
MAX_IMAGE_SIDE = 2048


@dataclass(frozen=True)
class ImageFrame:
    seq: int
    camera_ticks_ms: int
    width: int
    height: int
    jpeg: bytes


def encode_jpeg_frame(frame: ImageFrame) -> bytes:
    """Encode one test frame; firmware must use the same field order."""
    if not 1 <= frame.width <= MAX_IMAGE_SIDE:
        raise ValueError("invalid width")
    if not 1 <= frame.height <= MAX_IMAGE_SIDE:
        raise ValueError("invalid height")
    if not 2 <= len(frame.jpeg) <= MAX_JPEG_SIZE:
        raise ValueError("invalid JPEG size")
    if not frame.jpeg.startswith(b"\xff\xd8") or not frame.jpeg.endswith(b"\xff\xd9"):
        raise ValueError("missing JPEG SOI/EOI markers")
    if not 0 <= frame.seq <= 0xFFFFFFFF:
        raise ValueError("invalid sequence")
    if not 0 <= frame.camera_ticks_ms <= 0xFFFFFFFF:
        raise ValueError("invalid timestamp")
    header = HEADER.pack(
        MAGIC,
        VERSION,
        KIND_JPEG,
        frame.seq,
        frame.camera_ticks_ms,
        frame.width,
        frame.height,
        len(frame.jpeg),
        zlib.crc32(frame.jpeg) & 0xFFFFFFFF,
    )
    return header + frame.jpeg


class ImageFrameDecoder:
    """Incremental TCP/UART decoder; skips noise and corrupted frames."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_headers = 0
        self.bad_payloads = 0

    def feed(self, chunk: bytes) -> list[ImageFrame]:
        self._buffer.extend(chunk)
        frames: list[ImageFrame] = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                # Preserve a possible partial magic across read boundaries.
                del self._buffer[: max(0, len(self._buffer) - len(MAGIC) + 1)]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < HEADER.size:
                break
            magic, version, kind, seq, ticks, width, height, size, crc = HEADER.unpack_from(self._buffer)
            if (
                magic != MAGIC
                or version != VERSION
                or kind != KIND_JPEG
                or not 1 <= width <= MAX_IMAGE_SIDE
                or not 1 <= height <= MAX_IMAGE_SIDE
                or not 2 <= size <= MAX_JPEG_SIZE
            ):
                self.bad_headers += 1
                del self._buffer[0]
                continue
            end = HEADER.size + size
            if len(self._buffer) < end:
                break
            jpeg = bytes(self._buffer[HEADER.size:end])
            if (
                (zlib.crc32(jpeg) & 0xFFFFFFFF) != crc
                or not jpeg.startswith(b"\xff\xd8")
                or not jpeg.endswith(b"\xff\xd9")
            ):
                self.bad_payloads += 1
                # A false/corrupt header may contain another valid magic.
                del self._buffer[0]
                continue
            frames.append(ImageFrame(seq, ticks, width, height, jpeg))
            del self._buffer[:end]
        return frames


def encode_frame_ack(sequence: int) -> bytes:
    """Encode the PC's acknowledgement for a fully processed frame."""
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise ValueError("invalid sequence")
    return FRAME_ACK.pack(ACK_MAGIC, sequence)
