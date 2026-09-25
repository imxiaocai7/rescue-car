"""Versioned motion-command frames shared by the PC, ESP32-S3, and STM32."""

from dataclasses import dataclass
from enum import IntEnum
import struct


MAGIC = b"RCM1"
VERSION = 1
TYPE_COMMAND = 1
MAX_LEASE_MS = 1_000
UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000
PREFIX = struct.Struct("<4sBBIHhhBB")
CRC_FIELD = struct.Struct("<H")
FRAME_SIZE = PREFIX.size + CRC_FIELD.size


class ActuatorCommand(IntEnum):
    HOLD = 0
    CLOSE = 1
    OPEN = 2


@dataclass(frozen=True)
class MotionCommand:
    sequence: int
    lease_ms: int
    left_mm_s: int
    right_mm_s: int
    actuator: ActuatorCommand = ActuatorCommand.HOLD


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_motion_command(command: MotionCommand) -> bytes:
    """Encode one bounded command; speeds are signed left/right mm/s."""
    _validate_command(command)
    prefix = PREFIX.pack(
        MAGIC,
        VERSION,
        TYPE_COMMAND,
        command.sequence,
        command.lease_ms,
        command.left_mm_s,
        command.right_mm_s,
        int(command.actuator),
        0,
    )
    return prefix + CRC_FIELD.pack(crc16_ccitt_false(prefix))


def _validate_command(command: MotionCommand) -> None:
    if type(command.sequence) is not int or not 0 <= command.sequence <= UINT32_MASK:
        raise ValueError("sequence must be an unsigned 32-bit integer")
    if type(command.lease_ms) is not int or not 1 <= command.lease_ms <= MAX_LEASE_MS:
        raise ValueError("lease must be between 1 and 1000 ms")
    for name, value in (("left speed", command.left_mm_s), ("right speed", command.right_mm_s)):
        if type(value) is not int or not -32768 <= value <= 32767:
            raise ValueError(f"{name} must fit a signed 16-bit integer")
    if not isinstance(command.actuator, ActuatorCommand):
        raise ValueError("actuator must be a defined ActuatorCommand")


class MotionCommandDecoder:
    """Incremental fixed-size decoder with byte-wise recovery after corruption."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_headers = 0
        self.bad_checksums = 0

    def feed(self, chunk: bytes) -> list[MotionCommand]:
        self._buffer.extend(chunk)
        commands: list[MotionCommand] = []
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                del self._buffer[: max(0, len(self._buffer) - len(MAGIC) + 1)]
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                break

            prefix = bytes(self._buffer[:PREFIX.size])
            magic, version, message_type, sequence, lease_ms, left, right, actuator, reserved = PREFIX.unpack(prefix)
            if (
                magic != MAGIC
                or version != VERSION
                or message_type != TYPE_COMMAND
                or not 1 <= lease_ms <= MAX_LEASE_MS
                or actuator not in ActuatorCommand._value2member_map_
                or reserved != 0
            ):
                self.bad_headers += 1
                del self._buffer[0]
                continue

            expected_crc = CRC_FIELD.unpack_from(self._buffer, PREFIX.size)[0]
            if crc16_ccitt_false(prefix) != expected_crc:
                self.bad_checksums += 1
                del self._buffer[0]
                continue

            commands.append(
                MotionCommand(
                    sequence,
                    lease_ms,
                    left,
                    right,
                    ActuatorCommand(actuator),
                )
            )
            del self._buffer[:FRAME_SIZE]
        return commands


def sequence_is_newer(candidate: int, previous: int) -> bool:
    """Return true for a newer u32 sequence, including wrap from FFFFFFFF to 0."""
    _validate_u32(candidate, "candidate sequence")
    _validate_u32(previous, "previous sequence")
    delta = (candidate - previous) & UINT32_MASK
    return 0 < delta < UINT32_HALF_RANGE


def lease_expired(now_ms: int, received_ms: int, lease_ms: int) -> bool:
    """Wrap-safe millisecond watchdog; callers supply a monotonic u32 tick."""
    _validate_u32(now_ms, "current time")
    _validate_u32(received_ms, "receive time")
    if type(lease_ms) is not int or not 1 <= lease_ms <= MAX_LEASE_MS:
        raise ValueError("lease must be between 1 and 1000 ms")
    elapsed = (now_ms - received_ms) & UINT32_MASK
    return elapsed >= lease_ms


def _validate_u32(value: int, name: str) -> None:
    if type(value) is not int or not 0 <= value <= UINT32_MASK:
        raise ValueError(f"{name} must be an unsigned 32-bit integer")
