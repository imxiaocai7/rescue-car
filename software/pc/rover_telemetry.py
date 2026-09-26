"""STM32-to-PC telemetry frame format and incremental decoder."""

from dataclasses import dataclass
from enum import IntFlag
import struct

from motion_protocol import crc16_ccitt_false


MAGIC = b"RST1"
VERSION = 1
TYPE_TELEMETRY = 2
INVALID_RANGE_MM = 0xFFFF
PREFIX = struct.Struct("<4sBBIIIiihHHHH")
CRC_FIELD = struct.Struct("<H")
FRAME_SIZE = PREFIX.size + CRC_FIELD.size
UINT32_MAX = 0xFFFFFFFF
INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1


class TelemetryStatus(IntFlag):
    LEFT_ENCODER_VALID = 1 << 0
    RIGHT_ENCODER_VALID = 1 << 1
    GYRO_VALID = 1 << 2
    TOF_LEFT_VALID = 1 << 3
    TOF_RIGHT_VALID = 1 << 4
    BATTERY_VALID = 1 << 5
    SERVO_OPEN = 1 << 6
    SERVO_CLOSED = 1 << 7
    EMERGENCY_STOP = 1 << 8
    LAST_COMMAND_VALID = 1 << 9


KNOWN_STATUS_MASK = sum(int(flag) for flag in TelemetryStatus)


@dataclass(frozen=True)
class RoverTelemetry:
    sequence: int
    stm32_ticks_ms: int
    last_command_sequence: int
    left_encoder_total: int
    right_encoder_total: int
    gyro_z_mrad_s: int
    tof_left_mm: int
    tof_right_mm: int
    battery_mv: int
    status: TelemetryStatus


def encode_telemetry(frame: RoverTelemetry) -> bytes:
    _validate(frame)
    prefix = PREFIX.pack(
        MAGIC,
        VERSION,
        TYPE_TELEMETRY,
        frame.sequence,
        frame.stm32_ticks_ms,
        frame.last_command_sequence,
        frame.left_encoder_total,
        frame.right_encoder_total,
        frame.gyro_z_mrad_s,
        frame.tof_left_mm,
        frame.tof_right_mm,
        frame.battery_mv,
        int(frame.status),
    )
    return prefix + CRC_FIELD.pack(crc16_ccitt_false(prefix))


def _validate(frame: RoverTelemetry) -> None:
    if not isinstance(frame, RoverTelemetry):
        raise TypeError("frame must be RoverTelemetry")
    for name, value in (
        ("sequence", frame.sequence),
        ("stm32_ticks_ms", frame.stm32_ticks_ms),
        ("last_command_sequence", frame.last_command_sequence),
    ):
        if type(value) is not int or not 0 <= value <= UINT32_MAX:
            raise ValueError(f"{name} must be an unsigned 32-bit integer")
    for name, value in (
        ("left_encoder_total", frame.left_encoder_total),
        ("right_encoder_total", frame.right_encoder_total),
    ):
        if type(value) is not int or not INT32_MIN <= value <= INT32_MAX:
            raise ValueError(f"{name} must fit a signed 32-bit integer")
    if type(frame.gyro_z_mrad_s) is not int or not -32768 <= frame.gyro_z_mrad_s <= 32767:
        raise ValueError("gyro_z_mrad_s must fit a signed 16-bit integer")
    for name, value in (("tof_left_mm", frame.tof_left_mm), ("tof_right_mm", frame.tof_right_mm)):
        if type(value) is not int or not 0 <= value <= 0xFFFF:
            raise ValueError(f"{name} must fit an unsigned 16-bit integer")
    if type(frame.battery_mv) is not int or not 0 <= frame.battery_mv <= 0xFFFF:
        raise ValueError("battery_mv must fit an unsigned 16-bit integer")
    if not isinstance(frame.status, TelemetryStatus):
        raise ValueError("status must be a TelemetryStatus")
    flags = int(frame.status)
    if flags & ~KNOWN_STATUS_MASK:
        raise ValueError("reserved telemetry status bits must be zero")
    if bool(flags & TelemetryStatus.SERVO_OPEN) and bool(flags & TelemetryStatus.SERVO_CLOSED):
        raise ValueError("servo cannot be reported both open and closed")
    for flag, value, name in (
        (TelemetryStatus.TOF_LEFT_VALID, frame.tof_left_mm, "left ToF"),
        (TelemetryStatus.TOF_RIGHT_VALID, frame.tof_right_mm, "right ToF"),
    ):
        if flags & flag:
            if not 20 <= value <= 4000:
                raise ValueError(f"valid {name} reading must be 20-4000 mm")
        elif value != INVALID_RANGE_MM:
            raise ValueError(f"invalid {name} reading must use 0xFFFF")
    if flags & TelemetryStatus.BATTERY_VALID:
        if frame.battery_mv == 0:
            raise ValueError("valid battery reading must be nonzero")
    elif frame.battery_mv != 0:
        raise ValueError("invalid battery reading must use zero")
    if not flags & TelemetryStatus.GYRO_VALID and frame.gyro_z_mrad_s != 0:
        raise ValueError("invalid gyro reading must use zero")


class TelemetryDecoder:
    """Recover valid fixed-size telemetry frames from arbitrary TCP chunks."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_headers = 0
        self.bad_checksums = 0
        self.bad_payloads = 0

    def feed(self, chunk: bytes) -> list[RoverTelemetry]:
        self._buffer.extend(chunk)
        frames: list[RoverTelemetry] = []
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
            magic, version, kind, *_ = PREFIX.unpack(prefix)
            if magic != MAGIC or version != VERSION or kind != TYPE_TELEMETRY:
                self.bad_headers += 1
                del self._buffer[0]
                continue
            expected_crc = CRC_FIELD.unpack_from(self._buffer, PREFIX.size)[0]
            if crc16_ccitt_false(prefix) != expected_crc:
                self.bad_checksums += 1
                del self._buffer[0]
                continue

            values = PREFIX.unpack(prefix)
            try:
                frame = RoverTelemetry(
                    sequence=values[3],
                    stm32_ticks_ms=values[4],
                    last_command_sequence=values[5],
                    left_encoder_total=values[6],
                    right_encoder_total=values[7],
                    gyro_z_mrad_s=values[8],
                    tof_left_mm=values[9],
                    tof_right_mm=values[10],
                    battery_mv=values[11],
                    status=TelemetryStatus(values[12]),
                )
                _validate(frame)
            except (TypeError, ValueError):
                self.bad_payloads += 1
                del self._buffer[0]
                continue
            frames.append(frame)
            del self._buffer[:FRAME_SIZE]
        return frames
