"""Turn STM32 telemetry into a guarded differential-drive pose estimate."""

from dataclasses import dataclass, replace
import math

from odometry import (
    DifferentialDriveConfig,
    DifferentialDriveOdometry,
    Pose2D,
    modular_tick_delta,
)
from rover_telemetry import (
    KNOWN_STATUS_MASK,
    RoverTelemetry,
    TelemetryStatus,
)


UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000
ENCODER_COUNTER_MODULUS = 1 << 32
TAU = 2.0 * math.pi


@dataclass(frozen=True)
class TelemetryOdometryConfig:
    drive: DifferentialDriveConfig
    max_wheel_speed_m_s: float
    max_sample_gap_ms: int = 300
    max_pose_age_ms: int = 250
    left_encoder_sign: int = 1
    right_encoder_sign: int = 1
    gyro_sign: int = 1
    gyro_bias_mrad_s: float = 0.0

    def __post_init__(self) -> None:
        if not isinstance(self.drive, DifferentialDriveConfig):
            raise ValueError("drive must be a DifferentialDriveConfig")
        if not math.isfinite(self.max_wheel_speed_m_s) or self.max_wheel_speed_m_s <= 0:
            raise ValueError("max_wheel_speed_m_s must be positive and finite")
        for name, value in (
            ("max_sample_gap_ms", self.max_sample_gap_ms),
            ("max_pose_age_ms", self.max_pose_age_ms),
        ):
            if type(value) is not int or not 1 <= value < UINT32_HALF_RANGE:
                raise ValueError(f"{name} must be below the u32 half range")
        for name, value in (
            ("left_encoder_sign", self.left_encoder_sign),
            ("right_encoder_sign", self.right_encoder_sign),
            ("gyro_sign", self.gyro_sign),
        ):
            if type(value) is not int or value not in (-1, 1):
                raise ValueError(f"{name} must be -1 or 1")
        if not math.isfinite(self.gyro_bias_mrad_s):
            raise ValueError("gyro_bias_mrad_s must be finite")


@dataclass(frozen=True)
class PoseEstimate:
    pose: Pose2D
    valid: bool
    status: str
    received_at_ms: int | None
    telemetry_sequence: int | None
    dt_seconds: float | None = None


class TelemetryOdometry:
    """Integrate fresh, ordered RST1 telemetry, stopping on uncertain odometry.

    The initial pose is an externally established field pose. If a sensor gap,
    impossible wheel jump, or encoder-invalid sample makes dead reckoning
    uncertain, call ``reset_to_pose`` after a reliable relocalization; merely
    receiving later telemetry does not silently claim the pose recovered.
    """

    def __init__(
        self,
        config: TelemetryOdometryConfig,
        initial_pose: Pose2D = Pose2D(),
    ) -> None:
        if not isinstance(config, TelemetryOdometryConfig):
            raise TypeError("config must be a TelemetryOdometryConfig")
        _validate_pose(initial_pose)
        self.config = config
        # RST1 totals are signed views of a modulo-u32 counter. Maintain
        # monotonic virtual totals below, so the core odometer needs no wrap.
        core_config = replace(config.drive, encoder_modulus=None)
        self._odometry = DifferentialDriveOdometry(core_config, initial_pose)
        self._previous: RoverTelemetry | None = None
        self._last_sequence: int | None = None
        self._last_received_ms: int | None = None
        self._virtual_left = 0
        self._virtual_right = 0
        self._valid = False
        self._requires_relocalization = False
        self._status = "waiting_for_telemetry"

    @property
    def pose(self) -> Pose2D:
        return self._odometry.pose

    def update(self, frame: RoverTelemetry, *, received_at_ms: int) -> PoseEstimate:
        _validate_u32(received_at_ms, "received_at_ms")
        if not isinstance(frame, RoverTelemetry):
            raise TypeError("frame must be RoverTelemetry")
        _validate_telemetry(frame)

        if self._last_sequence is not None and not _is_newer_u32(
            frame.sequence, self._last_sequence
        ):
            return self.snapshot(received_at_ms, status="replayed_or_old_telemetry")
        self._last_sequence = frame.sequence

        required = (
            TelemetryStatus.LEFT_ENCODER_VALID
            | TelemetryStatus.RIGHT_ENCODER_VALID
        )
        if frame.status & required != required:
            self._invalidate("encoder_telemetry_invalid")
            return self.snapshot(received_at_ms)

        if self._previous is None:
            if self._requires_relocalization:
                self._status = "relocalization_required"
                return self.snapshot(received_at_ms)
            self._seed(frame, received_at_ms)
            self._status = "seeded_at_configured_pose"
            return self.snapshot(received_at_ms)

        tick_delta = (frame.stm32_ticks_ms - self._previous.stm32_ticks_ms) & UINT32_MASK
        if tick_delta == 0 or tick_delta >= UINT32_HALF_RANGE:
            self._invalidate("invalid_stm32_time_delta")
            return self.snapshot(received_at_ms)
        if tick_delta > self.config.max_sample_gap_ms:
            self._invalidate("telemetry_gap_requires_relocalization")
            return self.snapshot(received_at_ms)

        left_delta = modular_tick_delta(
            self._previous.left_encoder_total,
            frame.left_encoder_total,
            ENCODER_COUNTER_MODULUS,
        ) * self.config.left_encoder_sign
        right_delta = modular_tick_delta(
            self._previous.right_encoder_total,
            frame.right_encoder_total,
            ENCODER_COUNTER_MODULUS,
        ) * self.config.right_encoder_sign
        metres_per_tick = (
            TAU
            * self.config.drive.wheel_radius_m
            / self.config.drive.ticks_per_wheel_revolution
        )
        dt_seconds = tick_delta / 1000.0
        one_tick_margin_m = metres_per_tick
        maximum_distance_m = (
            self.config.max_wheel_speed_m_s * dt_seconds + one_tick_margin_m
        )
        if (
            abs(left_delta * metres_per_tick) > maximum_distance_m
            or abs(right_delta * metres_per_tick) > maximum_distance_m
        ):
            self._invalidate("implausible_encoder_jump")
            return self.snapshot(received_at_ms)

        self._virtual_left += left_delta
        self._virtual_right += right_delta
        gyro_rate = None
        if frame.status & TelemetryStatus.GYRO_VALID:
            gyro_rate = (
                (frame.gyro_z_mrad_s - self.config.gyro_bias_mrad_s)
                * self.config.gyro_sign
                / 1000.0
            )
        self._odometry.update(
            self._virtual_left,
            self._virtual_right,
            gyro_rate,
            dt_seconds,
        )
        self._previous = frame
        self._last_received_ms = received_at_ms
        self._valid = True
        self._status = "integrated" if gyro_rate is not None else "integrated_wheel_yaw_only"
        return self.snapshot(received_at_ms, dt_seconds=dt_seconds)

    def reset_to_pose(
        self,
        pose: Pose2D,
        *,
        telemetry: RoverTelemetry | None = None,
        received_at_ms: int | None = None,
    ) -> PoseEstimate:
        """Re-anchor to a known field pose, optionally seeding current counters."""
        _validate_pose(pose)
        if (telemetry is None) != (received_at_ms is None):
            raise ValueError("telemetry and received_at_ms must be supplied together")
        if telemetry is not None:
            _validate_telemetry(telemetry)
            _validate_u32(received_at_ms, "received_at_ms")
            required = (
                TelemetryStatus.LEFT_ENCODER_VALID
                | TelemetryStatus.RIGHT_ENCODER_VALID
            )
            if telemetry.status & required != required:
                raise ValueError("relocalization telemetry needs valid wheel encoders")

        self._odometry.reset(0, 0, pose)
        self._virtual_left = 0
        self._virtual_right = 0
        self._previous = None
        self._last_received_ms = None
        self._last_sequence = None
        self._valid = False
        self._requires_relocalization = False
        self._status = "relocalized_waiting_for_encoder_seed"
        if telemetry is not None:
            self._seed(telemetry, received_at_ms)
            self._status = "relocalized_and_seeded"
        return self.snapshot(received_at_ms)

    def snapshot(
        self,
        now_ms: int | None,
        *,
        status: str | None = None,
        dt_seconds: float | None = None,
    ) -> PoseEstimate:
        if now_ms is not None:
            _validate_u32(now_ms, "now_ms")
        fresh = (
            now_ms is not None
            and self._last_received_ms is not None
            and _is_fresh_u32(now_ms, self._last_received_ms, self.config.max_pose_age_ms)
        )
        estimate_status = status or self._status
        if self._valid and not fresh:
            estimate_status = "pose_stale"
        return PoseEstimate(
            pose=self._odometry.pose,
            valid=self._valid and fresh,
            status=estimate_status,
            received_at_ms=self._last_received_ms,
            telemetry_sequence=self._last_sequence,
            dt_seconds=dt_seconds,
        )

    def _seed(self, frame: RoverTelemetry, received_at_ms: int) -> None:
        self._previous = frame
        self._last_sequence = frame.sequence
        self._last_received_ms = received_at_ms
        self._virtual_left = 0
        self._virtual_right = 0
        self._odometry.reset(0, 0)
        self._valid = True

    def _invalidate(self, reason: str) -> None:
        self._valid = False
        self._requires_relocalization = True
        self._previous = None
        self._status = reason


def _validate_pose(pose: Pose2D) -> None:
    if not isinstance(pose, Pose2D) or not all(
        math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)
    ):
        raise ValueError("pose must contain finite x, y, and yaw")


def _validate_u32(value: int, name: str) -> None:
    if type(value) is not int or not 0 <= value <= UINT32_MASK:
        raise ValueError(f"{name} must be an unsigned 32-bit integer")


def _validate_telemetry(frame: RoverTelemetry) -> None:
    _validate_u32(frame.sequence, "telemetry sequence")
    _validate_u32(frame.stm32_ticks_ms, "STM32 tick")
    for name, value in (
        ("left encoder total", frame.left_encoder_total),
        ("right encoder total", frame.right_encoder_total),
    ):
        if type(value) is not int or not -(2**31) <= value < 2**31:
            raise ValueError(f"{name} must fit a signed 32-bit integer")
    if not isinstance(frame.status, TelemetryStatus):
        raise ValueError("telemetry status must be a TelemetryStatus")
    if int(frame.status) & ~KNOWN_STATUS_MASK:
        raise ValueError("reserved telemetry status bits must be zero")
    if type(frame.gyro_z_mrad_s) is not int or not -32768 <= frame.gyro_z_mrad_s <= 32767:
        raise ValueError("gyro rate must fit a signed 16-bit integer")


def _is_newer_u32(candidate: int, previous: int) -> bool:
    delta = (candidate - previous) & UINT32_MASK
    return delta != 0 and delta < UINT32_HALF_RANGE


def _is_fresh_u32(now_ms: int, received_ms: int, max_age_ms: int) -> bool:
    age = (now_ms - received_ms) & UINT32_MASK
    return age < UINT32_HALF_RANGE and age <= max_age_ms
