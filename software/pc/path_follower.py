"""Pure-pursuit path following in velocity space for a differential base."""

from dataclasses import dataclass
import math
from collections.abc import Sequence

from odometry import Pose2D, wrap_angle


Point2D = tuple[float, float]


@dataclass(frozen=True)
class FollowerConfig:
    lookahead_m: float
    linear_speed_m_s: float
    max_angular_speed_rad_s: float
    goal_tolerance_m: float = 0.05
    progress_search_points: int = 32

    def __post_init__(self) -> None:
        values = (
            self.lookahead_m,
            self.linear_speed_m_s,
            self.max_angular_speed_rad_s,
            self.goal_tolerance_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("follower parameters must be finite")
        if self.lookahead_m <= 0 or self.linear_speed_m_s <= 0:
            raise ValueError("lookahead and linear speed must be positive")
        if self.max_angular_speed_rad_s <= 0 or self.goal_tolerance_m < 0:
            raise ValueError("angular speed must be positive and tolerance nonnegative")
        if self.progress_search_points < 1:
            raise ValueError("progress_search_points must be positive")


@dataclass(frozen=True)
class VelocityCommand:
    linear_m_s: float
    angular_rad_s: float
    done: bool = False


@dataclass(frozen=True)
class WheelSpeedCommand:
    left_m_s: float
    right_m_s: float


def differential_wheel_speeds(
    command: VelocityCommand,
    track_width_m: float,
    max_wheel_speed_m_s: float,
) -> WheelSpeedCommand:
    """Convert body twist to wheel linear speeds and scale to a shared limit."""
    if not all(math.isfinite(value) for value in (command.linear_m_s, command.angular_rad_s)):
        raise ValueError("velocity command must be finite")
    if not math.isfinite(track_width_m) or track_width_m <= 0:
        raise ValueError("track width must be positive and finite")
    if not math.isfinite(max_wheel_speed_m_s) or max_wheel_speed_m_s <= 0:
        raise ValueError("max wheel speed must be positive and finite")
    left = command.linear_m_s - command.angular_rad_s * track_width_m / 2.0
    right = command.linear_m_s + command.angular_rad_s * track_width_m / 2.0
    peak = max(abs(left), abs(right))
    if peak > max_wheel_speed_m_s:
        scale = max_wheel_speed_m_s / peak
        left *= scale
        right *= scale
    return WheelSpeedCommand(left, right)


class PurePursuitFollower:
    """Follow a polyline with a monotonic path cursor and bounded angular rate."""

    def __init__(self, config: FollowerConfig):
        self.config = config
        self._path: tuple[Point2D, ...] = ()
        self._cursor = 0

    def set_path(self, path: Sequence[Point2D]) -> None:
        points = tuple((float(x), float(y)) for x, y in path)
        if any(not math.isfinite(value) for point in points for value in point):
            raise ValueError("path coordinates must be finite")
        self._path = points
        self._cursor = 0

    def update(self, pose: Pose2D) -> VelocityCommand:
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError("pose must be finite")
        if not self._path:
            return VelocityCommand(0.0, 0.0, done=True)

        goal_x, goal_y = self._path[-1]
        if math.hypot(goal_x - pose.x, goal_y - pose.y) <= self.config.goal_tolerance_m:
            self._cursor = len(self._path) - 1
            return VelocityCommand(0.0, 0.0, done=True)

        self._advance_cursor(pose)
        target = self._lookahead_point()
        dx = target[0] - pose.x
        dy = target[1] - pose.y
        target_distance = math.hypot(dx, dy)
        if target_distance <= 1e-9:
            return VelocityCommand(0.0, 0.0)

        target_heading = math.atan2(dy, dx)
        alpha = wrap_angle(target_heading - pose.yaw)
        max_angular = self.config.max_angular_speed_rad_s

        # A forward-only base cannot reach a waypoint behind it by driving
        # straight; rotate first, while retaining the robot's safe speed limit.
        if abs(alpha) > math.pi / 2.0:
            return VelocityCommand(0.0, math.copysign(max_angular, alpha))

        curvature = 2.0 * math.sin(alpha) / target_distance
        linear = self.config.linear_speed_m_s
        angular = linear * curvature
        if abs(angular) > max_angular:
            scale = max_angular / abs(angular)
            linear *= scale
            angular *= scale
        return VelocityCommand(linear, angular)

    def _advance_cursor(self, pose: Pose2D) -> None:
        start = max(0, self._cursor - 1)
        stop = min(len(self._path), self._cursor + self.config.progress_search_points + 1)
        nearest = min(
            range(start, stop),
            key=lambda index: math.hypot(
                self._path[index][0] - pose.x,
                self._path[index][1] - pose.y,
            ),
        )
        self._cursor = max(self._cursor, nearest)

    def _lookahead_point(self) -> Point2D:
        accumulated = 0.0
        for index in range(self._cursor, len(self._path) - 1):
            start = self._path[index]
            end = self._path[index + 1]
            segment = math.hypot(end[0] - start[0], end[1] - start[1])
            if segment <= 1e-12:
                continue
            if accumulated + segment >= self.config.lookahead_m:
                fraction = (self.config.lookahead_m - accumulated) / segment
                return (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
            accumulated += segment
        return self._path[-1]
