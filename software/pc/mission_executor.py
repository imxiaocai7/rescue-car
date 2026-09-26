"""Rule-safe mission execution above the hardware-independent navigation core.

Inputs are ground-projected targets and obstacle circles in field coordinates.
Pickup and delivery are confirmed by explicit external events; the executor
never increments mission progress merely because the robot reached a waypoint.
"""

from dataclasses import dataclass, replace
from enum import Enum
import math
from collections.abc import Sequence

from autonomous_planner import AutonomousPlanner
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


UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


class MissionState(str, Enum):
    SEARCH = "search"
    NAVIGATE_TO_TARGET = "navigate_to_target"
    ACQUIRE = "acquire"
    RETURN_TO_ZONE = "return_to_zone"
    DELIVER = "deliver"


class PickupOutcome(str, Enum):
    NONE = "none"
    CONFIRMED = "confirmed"
    FAILED = "failed"


class DeliveryOutcome(str, Enum):
    NONE = "none"
    CONFIRMED = "confirmed"
    FAILED = "failed"


@dataclass(frozen=True)
class MissionExecutorConfig:
    material_zone_xy: tuple[float, float]
    casualty_zone_xy: tuple[float, float]
    pickup_standoff_m: float
    zone_tolerance_m: float
    obstacle_inflation_m: float
    vision_max_age_ms: int
    pose_max_age_ms: int
    command_lease_ms: int
    track_width_m: float
    max_wheel_speed_m_s: float

    def __post_init__(self) -> None:
        for name, point in (
            ("material_zone_xy", self.material_zone_xy),
            ("casualty_zone_xy", self.casualty_zone_xy),
        ):
            if len(point) != 2 or not all(math.isfinite(value) for value in point):
                raise ValueError(f"{name} must contain two finite coordinates")
        for name, value in (
            ("pickup_standoff_m", self.pickup_standoff_m),
            ("zone_tolerance_m", self.zone_tolerance_m),
            ("track_width_m", self.track_width_m),
            ("max_wheel_speed_m_s", self.max_wheel_speed_m_s),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        if self.max_wheel_speed_m_s > 32.767:
            raise ValueError("wheel speed would exceed the protocol int16 mm/s range")
        if not math.isfinite(self.obstacle_inflation_m) or self.obstacle_inflation_m < 0:
            raise ValueError("obstacle inflation must be finite and nonnegative")
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
class MissionExecutionResult:
    state: MissionState
    status: str
    command: MotionCommand
    frame: bytes
    progress: MissionProgress
    active_target: Target | None = None
    destination_xy: tuple[float, float] | None = None
    waypoints_m: tuple[tuple[float, float], ...] = ()
    route_length_m: float = 0.0


class MissionExecutor:
    """Run one fail-closed mission step and return a leased motor/servo frame.

    Target IDs must be stable across adjacent vision frames. A selected target
    that disappears causes a stop until it is observed again. Dynamic obstacles
    are ``(x, y, radius)`` circles in metres and are rebuilt into a temporary
    map on every step. The caller supplies pickup/drop-off confirmations from
    a sensor or operator; these events are not inferred from location alone.
    """

    def __init__(
        self,
        static_grid: GridMap,
        follower_config: FollowerConfig,
        config: MissionExecutorConfig,
        *,
        mission_policy: MissionPolicy | None = None,
        initial_progress: MissionProgress = MissionProgress(),
        diagonal: bool = True,
    ) -> None:
        self.static_grid = static_grid
        self.config = config
        self.mission_policy = mission_policy or MissionPolicy()
        self.progress = initial_progress
        self.diagonal = diagonal
        self._state = MissionState.SEARCH
        self._active_target: Target | None = None
        self._collected_target_ids: set[str] = set()
        self._trip_kinds: list[TargetKind] = []
        self._target_follower = PurePursuitFollower(
            replace(follower_config, goal_tolerance_m=config.pickup_standoff_m)
        )
        self._zone_follower = PurePursuitFollower(
            replace(follower_config, goal_tolerance_m=config.zone_tolerance_m)
        )
        self._target_path: tuple[tuple[float, float], ...] = ()
        self._zone_path: tuple[tuple[float, float], ...] = ()

        # Fail early on a misspelled/out-of-field home-zone coordinate.
        self.static_grid.world_to_cell(*config.material_zone_xy)
        self.static_grid.world_to_cell(*config.casualty_zone_xy)

    @property
    def state(self) -> MissionState:
        return self._state

    def step(
        self,
        command_sequence: int,
        targets: Sequence[Target],
        *,
        now_ms: int,
        vision_received_ms: int,
        pose: Pose2D | None,
        pose_received_ms: int | None,
        obstacles: Sequence[tuple[float, float, float]] = (),
        pickup_outcome: PickupOutcome = PickupOutcome.NONE,
        delivery_outcome: DeliveryOutcome = DeliveryOutcome.NONE,
        emergency_stop: bool = False,
    ) -> MissionExecutionResult:
        """Advance one state transition or tracking update, without I/O."""
        self._validate_u32(command_sequence, "command sequence")
        if not isinstance(pickup_outcome, PickupOutcome):
            raise TypeError("pickup_outcome must be a PickupOutcome")
        if not isinstance(delivery_outcome, DeliveryOutcome):
            raise TypeError("delivery_outcome must be a DeliveryOutcome")
        if emergency_stop:
            return self._stopped("emergency_stop", command_sequence)
        if not self._fresh(now_ms, vision_received_ms, self.config.vision_max_age_ms):
            return self._stopped("vision_stale", command_sequence)
        if (
            pose is None
            or pose_received_ms is None
            or not self._fresh(now_ms, pose_received_ms, self.config.pose_max_age_ms)
            or not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))
        ):
            return self._stopped("pose_stale_or_invalid", command_sequence)

        try:
            planning_grid = self._planning_grid(obstacles)
            self.static_grid.world_to_cell(pose.x, pose.y)
        except (ValueError, IndexError, TypeError, OverflowError):
            return self._stopped("invalid_navigation_geometry", command_sequence)

        if self._state is MissionState.SEARCH:
            if self._has_cargo():
                self._state = MissionState.RETURN_TO_ZONE
            else:
                selected = self._select_target(targets, pose, planning_grid)
                if selected is None:
                    return self._stopped("waiting_for_legal_target", command_sequence)
                self._begin_target(selected)

        if self._state is MissionState.NAVIGATE_TO_TARGET:
            return self._navigate_to_target(
                command_sequence, targets, pose, planning_grid
            )

        if self._state is MissionState.ACQUIRE:
            return self._handle_acquire(
                command_sequence,
                targets,
                pose,
                planning_grid,
                pickup_outcome,
            )

        if self._state is MissionState.RETURN_TO_ZONE:
            if not self._has_cargo():
                self._state = MissionState.SEARCH
                return self._stopped("cargo_empty", command_sequence)
            return self._navigate_to_zone(command_sequence, pose, planning_grid)

        if self._state is MissionState.DELIVER:
            return self._handle_delivery(command_sequence, delivery_outcome)

        return self._stopped("unknown_mission_state", command_sequence)

    def _select_target(
        self,
        targets: Sequence[Target],
        pose: Pose2D,
        grid: GridMap,
    ) -> Target | None:
        candidates = [
            target for target in targets
            if target.target_id not in self._collected_target_ids
        ]
        routes = AutonomousPlanner(
            grid,
            self.mission_policy,
            diagonal=self.diagonal,
        ).rank_targets(candidates, (pose.x, pose.y), self.progress)
        return routes[0].target if routes else None

    def _begin_target(self, target: Target) -> None:
        self._active_target = target
        self._target_path = ()
        self._state = MissionState.NAVIGATE_TO_TARGET

    def _navigate_to_target(
        self,
        sequence: int,
        targets: Sequence[Target],
        pose: Pose2D,
        grid: GridMap,
    ) -> MissionExecutionResult:
        observed = next(
            (
                target for target in targets
                if self._active_target is not None
                and target.target_id == self._active_target.target_id
            ),
            None,
        )
        if observed is None:
            return self._stopped("active_target_not_visible", sequence)
        if not self.mission_policy.evaluate(observed, self.progress).eligible:
            self._active_target = None
            self._target_path = ()
            self._state = MissionState.SEARCH
            return self._stopped("active_target_no_longer_legal", sequence)
        self._active_target = observed

        try:
            route = AutonomousPlanner(
                grid,
                self.mission_policy,
                diagonal=self.diagonal,
            ).plan_next_target((observed,), (pose.x, pose.y), self.progress)
        except (IndexError, ValueError):
            route = None
        if route is None:
            self._target_path = ()
            return self._stopped("target_route_unavailable", sequence)

        if route.waypoints_m != self._target_path:
            self._target_follower.set_path(route.waypoints_m)
            self._target_path = route.waypoints_m
        velocity = self._target_follower.update(pose)
        if velocity.done:
            self._state = MissionState.ACQUIRE
            return self._stopped(
                "awaiting_pickup_confirmation",
                sequence,
                actuator=ActuatorCommand.OPEN,
                target=observed,
                waypoints=route.waypoints_m,
                route_length=route.route_length_m,
            )
        return self._moving(
            "approaching_target",
            sequence,
            velocity,
            target=observed,
            waypoints=route.waypoints_m,
            route_length=route.route_length_m,
        )

    def _handle_acquire(
        self,
        sequence: int,
        targets: Sequence[Target],
        pose: Pose2D,
        grid: GridMap,
        outcome: PickupOutcome,
    ) -> MissionExecutionResult:
        if outcome is PickupOutcome.NONE:
            return self._stopped(
                "awaiting_pickup_confirmation",
                sequence,
                actuator=ActuatorCommand.OPEN,
                target=self._active_target,
            )
        if outcome is PickupOutcome.FAILED:
            self._state = MissionState.SEARCH
            self._active_target = None
            self._target_path = ()
            return self._stopped(
                "pickup_failed_retry_allowed",
                sequence,
                actuator=ActuatorCommand.CLOSE,
            )

        target = self._active_target
        if target is None:
            return self._stopped("pickup_confirmation_without_target", sequence)
        if not self.mission_policy.evaluate(target, self.progress).eligible:
            return self._stopped("pickup_confirmation_rejected_by_policy", sequence)
        if target.kind is TargetKind.CASUALTY:
            self.progress = replace(self.progress, casualty_onboard=True)
        elif target.kind in (TargetKind.ORDINARY, TargetKind.CORE):
            self.progress = replace(
                self.progress,
                carried_material_count=self.progress.carried_material_count + 1,
            )
        else:
            return self._stopped("non_collectible_pickup_rejected", sequence)
        self._collected_target_ids.add(target.target_id)
        self._trip_kinds.append(target.kind)
        self._active_target = None
        self._target_path = ()

        if (
            self.progress.casualty_onboard
            or not self.progress.first_ordinary_delivered
            or self.progress.carried_material_count >= self.mission_policy.max_materials
        ):
            self._state = MissionState.RETURN_TO_ZONE
            return self._stopped(
                "pickup_confirmed_returning",
                sequence,
                actuator=ActuatorCommand.CLOSE,
            )

        next_target = self._select_target(targets, pose, grid)
        if next_target is None:
            self._state = MissionState.RETURN_TO_ZONE
            return self._stopped(
                "pickup_confirmed_no_more_reachable_cargo",
                sequence,
                actuator=ActuatorCommand.CLOSE,
                target=target,
            )
        self._begin_target(next_target)
        return self._stopped(
            "pickup_confirmed_collecting_more",
            sequence,
            actuator=ActuatorCommand.CLOSE,
        )

    def _navigate_to_zone(
        self,
        sequence: int,
        pose: Pose2D,
        grid: GridMap,
    ) -> MissionExecutionResult:
        destination = (
            self.config.casualty_zone_xy
            if self.progress.casualty_onboard
            else self.config.material_zone_xy
        )
        try:
            path = tuple(
                grid.find_world_path(
                    (pose.x, pose.y),
                    destination,
                    diagonal=self.diagonal,
                )
                or ()
            )
        except (IndexError, ValueError):
            path = ()
        if not path:
            self._zone_path = ()
            return self._stopped(
                "drop_zone_route_unavailable",
                sequence,
                destination=destination,
            )
        if path != self._zone_path:
            self._zone_follower.set_path(path)
            self._zone_path = path
        velocity = self._zone_follower.update(pose)
        if velocity.done:
            self._state = MissionState.DELIVER
            self._zone_path = ()
            return self._stopped(
                "awaiting_delivery_confirmation",
                sequence,
                actuator=ActuatorCommand.OPEN,
                destination=destination,
                waypoints=path,
            )
        return self._moving(
            "returning_to_zone",
            sequence,
            velocity,
            destination=destination,
            waypoints=path,
        )

    def _handle_delivery(
        self,
        sequence: int,
        outcome: DeliveryOutcome,
    ) -> MissionExecutionResult:
        destination = (
            self.config.casualty_zone_xy
            if self.progress.casualty_onboard
            else self.config.material_zone_xy
        )
        if outcome is DeliveryOutcome.NONE:
            return self._stopped(
                "awaiting_delivery_confirmation",
                sequence,
                actuator=ActuatorCommand.OPEN,
                destination=destination,
            )
        if outcome is DeliveryOutcome.FAILED:
            return self._stopped(
                "delivery_not_confirmed_retry_allowed",
                sequence,
                actuator=ActuatorCommand.OPEN,
                destination=destination,
            )

        if self.progress.casualty_onboard:
            self.progress = replace(
                self.progress,
                casualty_onboard=False,
                delivered_casualty_count=self.progress.delivered_casualty_count + 1,
            )
        elif self.progress.carried_material_count:
            count = self.progress.carried_material_count
            if not self.progress.first_ordinary_delivered:
                if count != 1 or self._trip_kinds != [TargetKind.ORDINARY]:
                    return self._stopped("first_delivery_rule_violation", sequence)
                self.progress = replace(
                    self.progress,
                    first_ordinary_delivered=True,
                    delivered_material_count=self.progress.delivered_material_count + count,
                    carried_material_count=0,
                )
            else:
                self.progress = replace(
                    self.progress,
                    delivered_material_count=self.progress.delivered_material_count + count,
                    carried_material_count=0,
                )
        else:
            return self._stopped("delivery_confirmation_without_cargo", sequence)

        self._trip_kinds.clear()
        self._active_target = None
        self._target_path = ()
        self._zone_path = ()
        self._state = MissionState.SEARCH
        return self._stopped(
            "delivery_confirmed_searching",
            sequence,
            actuator=ActuatorCommand.CLOSE,
            destination=destination,
        )

    def _planning_grid(
        self,
        obstacles: Sequence[tuple[float, float, float]],
    ) -> GridMap:
        grid = self._copy_static_grid()
        for obstacle in obstacles:
            if len(obstacle) != 3:
                raise ValueError("each obstacle must contain x, y, and radius")
            x, y, radius = obstacle
            grid.add_circle_obstacle(
                x,
                y,
                radius,
                inflation_m=self.config.obstacle_inflation_m,
            )
        return grid

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

    def _has_cargo(self) -> bool:
        return self.progress.casualty_onboard or self.progress.carried_material_count > 0

    def _moving(
        self,
        status: str,
        sequence: int,
        velocity: VelocityCommand,
        *,
        target: Target | None = None,
        destination: tuple[float, float] | None = None,
        waypoints: tuple[tuple[float, float], ...] = (),
        route_length: float = 0.0,
    ) -> MissionExecutionResult:
        wheels = differential_wheel_speeds(
            velocity,
            self.config.track_width_m,
            self.config.max_wheel_speed_m_s,
        )
        return self._result(
            status,
            sequence,
            round(wheels.left_m_s * 1000.0),
            round(wheels.right_m_s * 1000.0),
            ActuatorCommand.HOLD,
            target,
            destination,
            waypoints,
            route_length,
        )

    def _stopped(
        self,
        status: str,
        sequence: int,
        *,
        actuator: ActuatorCommand = ActuatorCommand.HOLD,
        target: Target | None = None,
        destination: tuple[float, float] | None = None,
        waypoints: tuple[tuple[float, float], ...] = (),
        route_length: float = 0.0,
    ) -> MissionExecutionResult:
        return self._result(
            status,
            sequence,
            0,
            0,
            actuator,
            target,
            destination,
            waypoints,
            route_length,
        )

    def _result(
        self,
        status: str,
        sequence: int,
        left_mm_s: int,
        right_mm_s: int,
        actuator: ActuatorCommand,
        target: Target | None,
        destination: tuple[float, float] | None,
        waypoints: tuple[tuple[float, float], ...],
        route_length: float,
    ) -> MissionExecutionResult:
        command = MotionCommand(
            sequence=sequence,
            lease_ms=self.config.command_lease_ms,
            left_mm_s=left_mm_s,
            right_mm_s=right_mm_s,
            actuator=actuator,
        )
        return MissionExecutionResult(
            state=self._state,
            status=status,
            command=command,
            frame=encode_motion_command(command),
            progress=self.progress,
            active_target=target or self._active_target,
            destination_xy=destination,
            waypoints_m=waypoints,
            route_length_m=route_length,
        )

    @staticmethod
    def _validate_u32(value: int, name: str) -> None:
        if type(value) is not int or not 0 <= value <= UINT32_MASK:
            raise ValueError(f"{name} must be an unsigned 32-bit integer")

    @classmethod
    def _fresh(cls, now_ms: int, stamp_ms: int, max_age_ms: int) -> bool:
        if type(now_ms) is not int or type(stamp_ms) is not int:
            return False
        if not 0 <= now_ms <= UINT32_MASK or not 0 <= stamp_ms <= UINT32_MASK:
            return False
        age = (now_ms - stamp_ms) & UINT32_MASK
        return age < UINT32_HALF_RANGE and age <= max_age_ms
