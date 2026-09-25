"""Fail-closed PC cycle connecting detections, routing, following, and wire frames."""

from dataclasses import dataclass, replace
import math
from collections.abc import Mapping, Sequence

from autonomous_planner import AutonomousPlanner, TargetRoute
from grid_planner import GridMap
from mission_policy import MissionPolicy, MissionProgress, Target, TargetKind
from motion_protocol import (
    ActuatorCommand,
    MAX_LEASE_MS,
    MotionCommand,
    encode_motion_command,
)
from odometry import Pose2D
from path_follower import (
    FollowerConfig,
    PurePursuitFollower,
    VelocityCommand,
    differential_wheel_speeds,
)
from pixel_ground import GroundPlaneProjector, robot_to_field
from rescue_vision import Detection


UINT32_MAX = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


@dataclass(frozen=True)
class AutonomyCycleConfig:
    class_kind_by_id: Mapping[int, TargetKind]
    obstacle_radius_m_by_class: Mapping[int, float]
    unknown_obstacle_radius_m: float
    obstacle_inflation_m: float
    vision_max_age_ms: int
    pose_max_age_ms: int
    command_lease_ms: int
    track_width_m: float
    max_wheel_speed_m_s: float
    target_standoff_m: float

    def __post_init__(self) -> None:
        if any(
            type(class_id) is not int or class_id < 0
            for class_id in self.class_kind_by_id
        ):
            raise ValueError("class ids must be nonnegative integers")
        if any(not isinstance(kind, TargetKind) for kind in self.class_kind_by_id.values()):
            raise ValueError("every mapped class must have a TargetKind")
        if any(
            type(class_id) is not int or class_id < 0
            for class_id in self.obstacle_radius_m_by_class
        ):
            raise ValueError("obstacle class ids must be nonnegative integers")
        for class_id, radius in self.obstacle_radius_m_by_class.items():
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError(f"obstacle radius for class {class_id} must be positive")
        for class_id, kind in self.class_kind_by_id.items():
            if kind in (TargetKind.DANGER, TargetKind.OPPONENT_ROBOT):
                if class_id not in self.obstacle_radius_m_by_class:
                    raise ValueError(f"obstacle radius missing for hazard class {class_id}")

        positive_distances = (
            self.unknown_obstacle_radius_m,
            self.track_width_m,
            self.max_wheel_speed_m_s,
            self.target_standoff_m,
        )
        if not all(math.isfinite(value) and value > 0 for value in positive_distances):
            raise ValueError("obstacle radius, wheel geometry, speed, and standoff must be positive")
        if not math.isfinite(self.obstacle_inflation_m) or self.obstacle_inflation_m < 0:
            raise ValueError("obstacle inflation must be finite and nonnegative")
        if self.max_wheel_speed_m_s > 32.767:
            raise ValueError("wheel speed would exceed the protocol int16 mm/s range")
        for name, value in (
            ("vision_max_age_ms", self.vision_max_age_ms),
            ("pose_max_age_ms", self.pose_max_age_ms),
        ):
            if type(value) is not int or not 1 <= value < UINT32_HALF_RANGE:
                raise ValueError(f"{name} must be a positive integer below the u32 half range")
        if (
            type(self.command_lease_ms) is not int
            or not 1 <= self.command_lease_ms <= MAX_LEASE_MS
        ):
            raise ValueError("command lease must be between 1 and 1000 ms")


@dataclass(frozen=True)
class AutonomyCycleResult:
    status: str
    motion_command: MotionCommand
    frame: bytes
    velocity: VelocityCommand
    target: Target | None = None
    route: TargetRoute | None = None
    projected_target_count: int = 0
    dynamic_obstacle_count: int = 0


class AutonomyCycle:
    """Run one deterministic planner cycle; never actuates hardware directly.

    Class IDs and object radii are caller-supplied because no trained rescue
    model or final CAD geometry is present. Unknown visual classes are treated
    as obstacles, not pickup targets. The grid is cloned before dynamic hazard
    detections are inserted so a frame cannot permanently mutate the static map.
    """

    def __init__(
        self,
        static_grid: GridMap,
        projector: GroundPlaneProjector | None,
        follower_config: FollowerConfig,
        config: AutonomyCycleConfig,
        *,
        mission_policy: MissionPolicy | None = None,
        diagonal: bool = True,
    ) -> None:
        self.static_grid = static_grid
        self.projector = projector
        self.config = config
        self.mission_policy = mission_policy or MissionPolicy()
        self.planner_diagonal = diagonal
        self.follower = PurePursuitFollower(
            replace(follower_config, goal_tolerance_m=config.target_standoff_m)
        )

    def run(
        self,
        command_sequence: int,
        detections: Sequence[Detection],
        *,
        now_ms: int,
        vision_received_ms: int,
        pose: Pose2D | None,
        pose_received_ms: int | None,
        progress: MissionProgress,
        emergency_stop: bool = False,
    ) -> AutonomyCycleResult:
        """Produce one leased velocity frame or a zero-speed fail-safe frame."""
        self._validate_u32(command_sequence, "command sequence")
        if emergency_stop:
            return self._stopped("emergency_stop", command_sequence)
        if not self._timestamp_fresh(now_ms, vision_received_ms, self.config.vision_max_age_ms):
            return self._stopped("vision_stale", command_sequence)
        if (
            pose is None
            or pose_received_ms is None
            or not self._timestamp_fresh(now_ms, pose_received_ms, self.config.pose_max_age_ms)
            or not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))
        ):
            return self._stopped("pose_stale_or_invalid", command_sequence)
        if self.projector is None:
            return self._stopped("ground_calibration_missing", command_sequence)

        targets: list[Target] = []
        obstacles: list[tuple[float, float, float]] = []
        id_counts: dict[tuple[int, int, int], int] = {}
        try:
            for detection in detections:
                local_xy = self.projector.box_footpoint_to_robot(detection.xyxy)
                field_xy = robot_to_field(local_xy, pose)
                kind = self.config.class_kind_by_id.get(detection.class_id)
                if kind is None:
                    radius = self.config.unknown_obstacle_radius_m
                    obstacles.append((field_xy[0], field_xy[1], radius))
                    continue

                if kind in (TargetKind.DANGER, TargetKind.OPPONENT_ROBOT):
                    obstacles.append(
                        (
                            field_xy[0],
                            field_xy[1],
                            self.config.obstacle_radius_m_by_class[detection.class_id],
                        )
                    )

                cell_x = math.floor((field_xy[0] - self.static_grid.origin_x) / self.static_grid.resolution_m)
                cell_y = math.floor((field_xy[1] - self.static_grid.origin_y) / self.static_grid.resolution_m)
                occurrence_key = (detection.class_id, cell_x, cell_y)
                occurrence = id_counts.get(occurrence_key, 0)
                id_counts[occurrence_key] = occurrence + 1
                targets.append(
                    Target(
                        target_id=f"{detection.class_id}:{cell_x}:{cell_y}:{occurrence}",
                        kind=kind,
                        x=field_xy[0],
                        y=field_xy[1],
                        confidence=detection.confidence,
                    )
                )
        except (ValueError, TypeError, OverflowError, AttributeError):
            return self._stopped("invalid_detection_geometry", command_sequence)

        planning_grid = self._copy_static_grid() if obstacles else self.static_grid
        for x, y, radius in obstacles:
            planning_grid.add_circle_obstacle(
                x,
                y,
                radius,
                inflation_m=self.config.obstacle_inflation_m,
            )

        planner = AutonomousPlanner(
            planning_grid,
            self.mission_policy,
            diagonal=self.planner_diagonal,
        )
        try:
            route = planner.plan_next_target(targets, (pose.x, pose.y), progress)
        except IndexError:
            self.follower.set_path(())
            return self._stopped(
                "pose_outside_map",
                command_sequence,
                projected_target_count=len(targets),
                dynamic_obstacle_count=len(obstacles),
            )
        if route is None:
            self.follower.set_path(())
            return self._stopped(
                "no_legal_reachable_target",
                command_sequence,
                projected_target_count=len(targets),
                dynamic_obstacle_count=len(obstacles),
            )

        self.follower.set_path(route.waypoints_m)
        velocity = self.follower.update(pose)
        wheels = differential_wheel_speeds(
            velocity,
            self.config.track_width_m,
            self.config.max_wheel_speed_m_s,
        )
        command = MotionCommand(
            sequence=command_sequence,
            lease_ms=self.config.command_lease_ms,
            left_mm_s=round(wheels.left_m_s * 1000.0),
            right_mm_s=round(wheels.right_m_s * 1000.0),
            actuator=ActuatorCommand.HOLD,
        )
        return AutonomyCycleResult(
            status="target_reached" if velocity.done else "tracking",
            motion_command=command,
            frame=encode_motion_command(command),
            velocity=velocity,
            target=route.target,
            route=route,
            projected_target_count=len(targets),
            dynamic_obstacle_count=len(obstacles),
        )

    def _stopped(
        self,
        status: str,
        sequence: int,
        *,
        projected_target_count: int = 0,
        dynamic_obstacle_count: int = 0,
    ) -> AutonomyCycleResult:
        command = MotionCommand(
            sequence=sequence,
            lease_ms=self.config.command_lease_ms,
            left_mm_s=0,
            right_mm_s=0,
            actuator=ActuatorCommand.HOLD,
        )
        return AutonomyCycleResult(
            status=status,
            motion_command=command,
            frame=encode_motion_command(command),
            velocity=VelocityCommand(0.0, 0.0, done=True),
            projected_target_count=projected_target_count,
            dynamic_obstacle_count=dynamic_obstacle_count,
        )

    def _copy_static_grid(self) -> GridMap:
        source = self.static_grid
        clone = GridMap(
            source.width_m,
            source.height_m,
            source.resolution_m,
            origin_x=source.origin_x,
            origin_y=source.origin_y,
        )
        for y in range(source.rows):
            for x in range(source.columns):
                if source.is_blocked((x, y)):
                    clone.set_blocked((x, y))
        return clone

    @staticmethod
    def _validate_u32(value: int, name: str) -> None:
        if type(value) is not int or not 0 <= value <= UINT32_MAX:
            raise ValueError(f"{name} must be an unsigned 32-bit integer")

    @classmethod
    def _timestamp_fresh(cls, now_ms: int, stamp_ms: int, max_age_ms: int) -> bool:
        if type(now_ms) is not int or type(stamp_ms) is not int:
            return False
        if not 0 <= now_ms <= UINT32_MAX or not 0 <= stamp_ms <= UINT32_MAX:
            return False
        age = (now_ms - stamp_ms) & UINT32_MAX
        return age < UINT32_HALF_RANGE and age <= max_age_ms
