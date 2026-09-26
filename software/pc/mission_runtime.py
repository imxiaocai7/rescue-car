"""Connect camera detections and odometry to the full mission executor."""

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math

from mission_executor import (
    DeliveryOutcome,
    MissionExecutionResult,
    MissionExecutor,
    PickupOutcome,
)
from mission_policy import Target, TargetKind
from odometry import Pose2D
from pixel_ground import GroundPlaneProjector, robot_to_field
from rescue_vision import Detection


UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000
COLLECTIBLE_KINDS = frozenset(
    {TargetKind.ORDINARY, TargetKind.CORE, TargetKind.CASUALTY}
)
HAZARD_KINDS = frozenset({TargetKind.DANGER, TargetKind.OPPONENT_ROBOT})


@dataclass(frozen=True)
class MissionPerceptionConfig:
    class_kind_by_id: Mapping[int, TargetKind]
    obstacle_radius_m_by_class: Mapping[int, float]
    unknown_obstacle_radius_m: float
    uncertain_target_radius_m: float
    minimum_target_confidence: float = 0.60
    minimum_obstacle_confidence: float = 0.15
    association_distance_m: float = 0.22
    track_timeout_ms: int = 500

    def __post_init__(self) -> None:
        for class_id, kind in self.class_kind_by_id.items():
            if type(class_id) is not int or class_id < 0:
                raise ValueError("class ids must be nonnegative integers")
            if not isinstance(kind, TargetKind):
                raise ValueError("each class mapping must use a TargetKind")
        for class_id, radius in self.obstacle_radius_m_by_class.items():
            if type(class_id) is not int or class_id < 0:
                raise ValueError("obstacle class ids must be nonnegative integers")
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError(f"obstacle radius for class {class_id} must be positive")
        for class_id, kind in self.class_kind_by_id.items():
            if kind in HAZARD_KINDS and class_id not in self.obstacle_radius_m_by_class:
                raise ValueError(f"obstacle radius missing for hazard class {class_id}")
        for name, value in (
            ("unknown_obstacle_radius_m", self.unknown_obstacle_radius_m),
            ("uncertain_target_radius_m", self.uncertain_target_radius_m),
            ("association_distance_m", self.association_distance_m),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name, value in (
            ("minimum_target_confidence", self.minimum_target_confidence),
            ("minimum_obstacle_confidence", self.minimum_obstacle_confidence),
        ):
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in [0, 1]")
        if type(self.track_timeout_ms) is not int or not 1 <= self.track_timeout_ms < UINT32_HALF_RANGE:
            raise ValueError("track_timeout_ms must be below the u32 half range")


@dataclass(frozen=True)
class ProjectedScene:
    targets: tuple[Target, ...]
    obstacles: tuple[tuple[float, float, float], ...]
    visible_target_count: int
    tracked_target_count: int
    ignored_detection_count: int


@dataclass(frozen=True)
class MissionRuntimeResult:
    execution: MissionExecutionResult
    scene: ProjectedScene


@dataclass
class _TargetTrack:
    target_id: str
    class_id: int
    kind: TargetKind
    x: float
    y: float
    confidence: float
    last_seen_ms: int

    def as_target(self) -> Target:
        return Target(
            self.target_id,
            self.kind,
            self.x,
            self.y,
            self.confidence,
        )


class DetectionProjectorTracker:
    """Project detections and give collectible objects stable short-term IDs.

    The track is a nearest-neighbour association in field coordinates. It is
    intentionally lightweight for the current low-rate camera pipeline; a
    stronger detector tracker can replace it without changing mission logic.
    Unseen tracks are retained for a bounded interval to bridge brief occlusion.
    """

    def __init__(
        self,
        projector: GroundPlaneProjector,
        config: MissionPerceptionConfig,
    ) -> None:
        self.projector = projector
        self.config = config
        self._tracks: dict[str, _TargetTrack] = {}
        self._next_track_number = 1

    def project(
        self,
        detections: Sequence[Detection],
        pose: Pose2D,
        *,
        observed_at_ms: int,
    ) -> ProjectedScene:
        _validate_u32(observed_at_ms, "observed_at_ms")
        if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            raise ValueError("pose must be finite before projecting detections")
        self._expire_tracks(observed_at_ms)

        obstacles: list[tuple[float, float, float]] = []
        measurements: list[tuple[int, TargetKind, float, float, float]] = []
        ignored = 0
        for detection in detections:
            try:
                if type(detection.class_id) is not int or detection.class_id < 0:
                    raise ValueError("detection class id must be nonnegative")
                if (
                    not math.isfinite(detection.confidence)
                    or not 0.0 <= detection.confidence <= 1.0
                ):
                    raise ValueError("detection confidence must be in [0, 1]")
                local_xy = self.projector.box_footpoint_to_robot(detection.xyxy)
                x, y = robot_to_field(local_xy, pose)
            except (ValueError, TypeError, OverflowError, AttributeError):
                ignored += 1
                continue

            kind = self.config.class_kind_by_id.get(detection.class_id)
            if kind is None:
                if detection.confidence >= self.config.minimum_obstacle_confidence:
                    obstacles.append(
                        (x, y, self.config.unknown_obstacle_radius_m)
                    )
                else:
                    ignored += 1
                continue

            if kind in HAZARD_KINDS:
                if detection.confidence >= self.config.minimum_obstacle_confidence:
                    obstacles.append(
                        (x, y, self.config.obstacle_radius_m_by_class[detection.class_id])
                    )
                else:
                    ignored += 1
                continue

            if kind not in COLLECTIBLE_KINDS:
                ignored += 1
                continue
            if detection.confidence < self.config.minimum_target_confidence:
                if detection.confidence >= self.config.minimum_obstacle_confidence:
                    obstacles.append((x, y, self.config.uncertain_target_radius_m))
                else:
                    ignored += 1
                continue
            measurements.append(
                (detection.class_id, kind, x, y, detection.confidence)
            )

        measurements.sort(key=lambda item: (item[0], item[2], item[3], -item[4]))
        matched_ids: set[str] = set()
        visible_ids: set[str] = set()
        for class_id, kind, x, y, confidence in measurements:
            track = self._match_track(class_id, x, y, matched_ids)
            if track is None:
                track_id = f"target-{self._next_track_number:06d}"
                self._next_track_number += 1
                track = _TargetTrack(
                    track_id, class_id, kind, x, y, confidence, observed_at_ms
                )
                self._tracks[track_id] = track
            else:
                track.x = x
                track.y = y
                track.confidence = confidence
                track.last_seen_ms = observed_at_ms
            matched_ids.add(track.target_id)
            visible_ids.add(track.target_id)

        all_tracks = sorted(self._tracks.values(), key=lambda item: item.target_id)
        return ProjectedScene(
            targets=tuple(track.as_target() for track in all_tracks),
            obstacles=tuple(obstacles),
            visible_target_count=len(visible_ids),
            tracked_target_count=len(all_tracks),
            ignored_detection_count=ignored,
        )

    def _match_track(
        self,
        class_id: int,
        x: float,
        y: float,
        already_matched: set[str],
    ) -> _TargetTrack | None:
        candidates: list[tuple[float, str, _TargetTrack]] = []
        limit_sq = self.config.association_distance_m**2
        for track in self._tracks.values():
            if track.class_id != class_id or track.target_id in already_matched:
                continue
            distance_sq = (track.x - x) ** 2 + (track.y - y) ** 2
            if distance_sq <= limit_sq:
                candidates.append((distance_sq, track.target_id, track))
        return min(candidates, key=lambda item: item[:2])[2] if candidates else None

    def _expire_tracks(self, now_ms: int) -> None:
        expired = []
        for track_id, track in self._tracks.items():
            age = (now_ms - track.last_seen_ms) & UINT32_MASK
            if age >= UINT32_HALF_RANGE or age > self.config.track_timeout_ms:
                expired.append(track_id)
        for track_id in expired:
            del self._tracks[track_id]


class RescueMissionRuntime:
    """Bridge YOLO detections, homography, target tracking, and mission output."""

    def __init__(
        self,
        executor: MissionExecutor,
        projector: GroundPlaneProjector,
        perception_config: MissionPerceptionConfig,
    ) -> None:
        self.executor = executor
        self.perception = DetectionProjectorTracker(projector, perception_config)

    def step(
        self,
        command_sequence: int,
        detections: Sequence[Detection],
        *,
        now_ms: int,
        vision_received_ms: int,
        pose: Pose2D | None,
        pose_received_ms: int | None,
        field_obstacles: Sequence[tuple[float, float, float]] = (),
        pickup_outcome: PickupOutcome = PickupOutcome.NONE,
        delivery_outcome: DeliveryOutcome = DeliveryOutcome.NONE,
        emergency_stop: bool = False,
    ) -> MissionRuntimeResult:
        fresh_vision = _is_fresh(
            now_ms, vision_received_ms, self.executor.config.vision_max_age_ms
        )
        fresh_pose = (
            pose is not None
            and pose_received_ms is not None
            and _is_fresh(now_ms, pose_received_ms, self.executor.config.pose_max_age_ms)
            and all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw))
        )
        if fresh_vision and fresh_pose and not emergency_stop:
            scene = self.perception.project(
                detections,
                pose,
                observed_at_ms=vision_received_ms,
            )
        else:
            scene = ProjectedScene((), (), 0, 0, 0)

        execution = self.executor.step(
            command_sequence,
            scene.targets,
            now_ms=now_ms,
            vision_received_ms=vision_received_ms,
            pose=pose,
            pose_received_ms=pose_received_ms,
            obstacles=(*scene.obstacles, *field_obstacles),
            pickup_outcome=pickup_outcome,
            delivery_outcome=delivery_outcome,
            emergency_stop=emergency_stop,
        )
        return MissionRuntimeResult(execution, scene)


def _validate_u32(value: int, name: str) -> None:
    if type(value) is not int or not 0 <= value <= UINT32_MASK:
        raise ValueError(f"{name} must be an unsigned 32-bit integer")


def _is_fresh(now_ms: int, stamp_ms: int, max_age_ms: int) -> bool:
    if type(now_ms) is not int or type(stamp_ms) is not int:
        return False
    if not 0 <= now_ms <= UINT32_MASK or not 0 <= stamp_ms <= UINT32_MASK:
        return False
    age = (now_ms - stamp_ms) & UINT32_MASK
    return age < UINT32_HALF_RANGE and age <= max_age_ms
