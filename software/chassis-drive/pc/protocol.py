"""New-project diagnostic wire format. Deliberately has no velocity opcode."""

from dataclasses import dataclass
from enum import IntEnum
import struct
import zlib

FIRMWARE_ID = 0x20261007
NO_MOTOR_IMPLEMENTATION = 1
MOTOR_FIRMWARE_ID = 0x20261011
MOTOR_IMPLEMENTED = 2


class Opcode(IntEnum):
    PING = 1
    STOP = 2


def u32(value):
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError("expected an unsigned 32-bit integer")
    return value


def checked_body(packet, magic, size):
    if len(packet) != size or packet[:4] != magic:
        raise ValueError("wrong frame size or magic")
    if struct.unpack("<I", packet[-4:])[0] != zlib.crc32(packet[:-4]):
        raise ValueError("CRC mismatch")
    return packet[:-4]


def with_crc(body):
    return body + struct.pack("<I", zlib.crc32(body))


@dataclass(frozen=True)
class Request:
    sequence: int
    opcode: Opcode = Opcode.PING

    def encode(self):
        return with_crc(struct.pack("<4sIB3x", b"CDP1", u32(self.sequence), Opcode(self.opcode)))

    @classmethod
    def decode(cls, packet):
        body = checked_body(packet, b"CDP1", 16)
        if body[9:] != b"\0\0\0":
            raise ValueError("reserved bytes must be zero")
        return cls(struct.unpack_from("<I", body, 4)[0], Opcode(body[8]))


@dataclass(frozen=True)
class Reply:
    sequence: int
    uptime_ms: int
    firmware_id: int = FIRMWARE_ID
    flags: int = NO_MOTOR_IMPLEMENTATION

    def encode(self):
        return with_crc(struct.pack("<4sIIII", b"CDS1", u32(self.sequence), u32(self.uptime_ms),
                                    u32(self.firmware_id), u32(self.flags)))

    @classmethod
    def decode(cls, packet):
        body = checked_body(packet, b"CDS1", 24)
        return cls(*struct.unpack_from("<IIII", body, 4))


class StreamParser:
    """Fixed-size sliding window; malformed/unbounded input cannot grow storage."""

    def __init__(self, frame_type):
        if frame_type not in (Request, Reply):
            raise ValueError("unsupported frame type")
        self.frame_type = frame_type
        self.size = 16 if frame_type is Request else 24
        self.buffer = bytearray()

    def feed(self, data):
        result = []
        for byte in data:
            self.buffer.append(byte)
            if len(self.buffer) < self.size:
                continue
            try:
                frame = self.frame_type.decode(self.buffer)
            except ValueError:
                del self.buffer[0]
            else:
                result.append(frame)
                self.buffer.clear()
        return result
