"""Two-wheel encoder odometry with complementary gyro heading fusion."""

from dataclasses import dataclass
import math
from collections.abc import Iterable


TAU = 2.0 * math.pi


@dataclass(frozen=True)
class Pose2D:
    """Robot center pose in a field frame: metres and counter-clockwise radians."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class DifferentialDriveConfig:
    wheel_radius_m: float
    track_width_m: float
    ticks_per_wheel_revolution: int
    gyro_weight: float = 0.98
    encoder_modulus: int | None = None

    def __post_init__(self) -> None:
        values = (self.wheel_radius_m, self.track_width_m, self.gyro_weight)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("odometry parameters must be finite")
        if self.wheel_radius_m <= 0 or self.track_width_m <= 0:
            raise ValueError("wheel radius and track width must be positive")
        if (
            isinstance(self.ticks_per_wheel_revolution, bool)
            or not isinstance(self.ticks_per_wheel_revolution, int)
            or self.ticks_per_wheel_revolution <= 0
        ):
            raise ValueError("ticks_per_wheel_revolution must be positive")
        if not 0.0 <= self.gyro_weight <= 1.0:
            raise ValueError("gyro_weight must be in [0, 1]")
        if self.encoder_modulus is not None and (
            isinstance(self.encoder_modulus, bool)
            or not isinstance(self.encoder_modulus, int)
            or self.encoder_modulus < 2
        ):
            raise ValueError("encoder_modulus must be an integer of at least 2")


def wrap_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    if not math.isfinite(angle_rad):
        raise ValueError("angle must be finite")
    return (angle_rad + math.pi) % TAU - math.pi


def modular_tick_delta(previous: int, current: int, modulus: int | None = None) -> int:
    """Return signed encoder delta, correcting a single counter wrap if configured."""
    if any(isinstance(value, bool) or not isinstance(value, int) for value in (previous, current)):
        raise ValueError("encoder counts must be integers")
    raw_delta = current - previous
    if modulus is None:
        return raw_delta
    if isinstance(modulus, bool) or not isinstance(modulus, int) or modulus < 2:
        raise ValueError("modulus must be an integer of at least 2")
    return (raw_delta + modulus // 2) % modulus - modulus // 2


def estimate_gyro_bias(samples_rad_s: Iterable[float]) -> float:
    """Estimate stationary Z-gyro bias as the mean of finite rad/s samples."""
    total = 0.0
    count = 0
    for sample in samples_rad_s:
        if not math.isfinite(sample):
            raise ValueError("gyro samples must be finite")
        total += sample
        count += 1
    if count == 0:
        raise ValueError("at least one stationary gyro sample is required")
    return total / count


class DifferentialDriveOdometry:
    """Integrate signed wheel ticks and bias-corrected BMI088 gyro rate.

    Both wheel tick streams must be positive in the robot's forward direction.
    The caller supplies gyro rate in rad/s after axis/sign and bias correction.
    This is dead reckoning; it does not claim global localization.
    """

    def __init__(self, config: DifferentialDriveConfig, pose: Pose2D = Pose2D()):
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError("pose must be finite")
        self.config = config
        self._x = pose.x
        self._y = pose.y
        self._yaw_unwrapped = pose.yaw
        self._previous_left: int | None = None
        self._previous_right: int | None = None

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self._x, self._y, wrap_angle(self._yaw_unwrapped))

    def reset(self, left_ticks: int, right_ticks: int, pose: Pose2D | None = None) -> Pose2D:
        """Set the starting pose and seed counters; the next update has no jump."""
        if pose is not None:
            if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
                raise ValueError("pose must be finite")
            self._x = pose.x
            self._y = pose.y
            self._yaw_unwrapped = pose.yaw
        self._previous_left = left_ticks
        self._previous_right = right_ticks
        return self.pose

    def update(
        self,
        left_ticks: int,
        right_ticks: int,
        gyro_z_rad_s: float,
        dt_seconds: float,
    ) -> Pose2D:
        if self._previous_left is None or self._previous_right is None:
            raise RuntimeError("call reset() with current encoder counters before update()")
        if not math.isfinite(gyro_z_rad_s):
            raise ValueError("gyro rate must be finite")
        if not math.isfinite(dt_seconds) or dt_seconds <= 0:
            raise ValueError("dt_seconds must be positive and finite")

        config = self.config
        left_delta = modular_tick_delta(
            self._previous_left, left_ticks, config.encoder_modulus
        )
        right_delta = modular_tick_delta(
            self._previous_right, right_ticks, config.encoder_modulus
        )
        self._previous_left = left_ticks
        self._previous_right = right_ticks

        metres_per_tick = TAU * config.wheel_radius_m / config.ticks_per_wheel_revolution
        left_distance = left_delta * metres_per_tick
        right_distance = right_delta * metres_per_tick
        wheel_yaw_delta = (right_distance - left_distance) / config.track_width_m
        gyro_yaw_delta = gyro_z_rad_s * dt_seconds
        yaw_delta = (
            config.gyro_weight * gyro_yaw_delta
            + (1.0 - config.gyro_weight) * wheel_yaw_delta
        )

        center_distance = (left_distance + right_distance) / 2.0
        midpoint_yaw = self._yaw_unwrapped + yaw_delta / 2.0
        self._x += center_distance * math.cos(midpoint_yaw)
        self._y += center_distance * math.sin(midpoint_yaw)
        self._yaw_unwrapped += yaw_delta
        return self.pose
