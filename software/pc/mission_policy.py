"""Rule-gated rescue target selection, independent of robot hardware."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable


class TargetKind(str, Enum):
    ORDINARY = "ordinary"
    CORE = "core"
    CASUALTY = "casualty"
    DANGER = "danger"
    OPPONENT_ROBOT = "opponent_robot"


MATERIAL_KINDS = frozenset({TargetKind.ORDINARY, TargetKind.CORE})
NON_COLLECTIBLE_KINDS = frozenset({TargetKind.DANGER, TargetKind.OPPONENT_ROBOT})


@dataclass(frozen=True)
class Target:
    """A classified target already projected into field coordinates (metres)."""

    target_id: str
    kind: TargetKind
    x: float
    y: float
    confidence: float

    def __post_init__(self) -> None:
        if not self.target_id:
            raise ValueError("target_id cannot be empty")
        if not isinstance(self.kind, TargetKind):
            raise TypeError("kind must be a TargetKind")
        if not math.isfinite(self.x) or not math.isfinite(self.y):
            raise ValueError("target coordinates must be finite")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")


@dataclass(frozen=True)
class MissionProgress:
    """Persistent rule state; update it only after confirmed pickup/drop-off.

    ``delivered_material_count`` is cumulative score progress, while
    ``carried_material_count`` is the number aboard on this trip. Capacity is
    per trip, so deliveries from earlier trips do not consume current capacity.
    """

    first_ordinary_delivered: bool = False
    delivered_material_count: int = 0
    carried_material_count: int = 0
    casualty_onboard: bool = False
    delivered_casualty_count: int = 0

    def __post_init__(self) -> None:
        for name, count in (
            ("delivered_material_count", self.delivered_material_count),
            ("carried_material_count", self.carried_material_count),
            ("delivered_casualty_count", self.delivered_casualty_count),
        ):
            if type(count) is not int or count < 0:
                raise ValueError(f"{name} must be a nonnegative integer")
        if type(self.first_ordinary_delivered) is not bool or type(self.casualty_onboard) is not bool:
            raise TypeError("mission flags must be booleans")
        if self.casualty_onboard and self.carried_material_count:
            raise ValueError("casualty and material cargo cannot share a trip")
        if self.casualty_onboard and not self.first_ordinary_delivered:
            raise ValueError("casualty tasks are locked until the first ordinary delivery")
        if not self.first_ordinary_delivered and self.delivered_material_count:
            raise ValueError("material delivery count cannot advance before the first ordinary delivery")
        if self.first_ordinary_delivered and self.delivered_material_count < 1:
            raise ValueError("first ordinary delivery must count as one delivered material")


@dataclass(frozen=True)
class TargetEvaluation:
    eligible: bool
    reason: str


class MissionPolicy:
    """Conservative collection policy from the current project plan.

    This module selects among already localized candidates. It does not infer
    classes, estimate coordinates, assign drop-off zones, or command motion.
    """

    def __init__(self, *, minimum_confidence: float = 0.60, max_materials: int = 3):
        if not 0.0 <= minimum_confidence <= 1.0:
            raise ValueError("minimum_confidence must be in [0, 1]")
        if max_materials < 1:
            raise ValueError("max_materials must be at least 1")
        self.minimum_confidence = minimum_confidence
        self.max_materials = max_materials

    def evaluate(self, target: Target, progress: MissionProgress) -> TargetEvaluation:
        if target.confidence < self.minimum_confidence:
            return TargetEvaluation(False, "low_confidence")
        if target.kind in NON_COLLECTIBLE_KINDS:
            return TargetEvaluation(False, "obstacle_only")
        if progress.casualty_onboard:
            return TargetEvaluation(False, "casualty_trip_is_exclusive")

        if not progress.first_ordinary_delivered:
            if target.kind is not TargetKind.ORDINARY:
                return TargetEvaluation(False, "first_delivery_requires_ordinary")
            if progress.delivered_material_count or progress.carried_material_count:
                return TargetEvaluation(False, "first_delivery_allows_exactly_one")
            return TargetEvaluation(True, "first_ordinary")

        if target.kind is TargetKind.CASUALTY:
            if progress.carried_material_count:
                return TargetEvaluation(False, "casualty_requires_empty_material_cargo")
            return TargetEvaluation(True, "casualty_trip")

        if target.kind in MATERIAL_KINDS:
            if progress.carried_material_count >= self.max_materials:
                return TargetEvaluation(False, "material_limit_reached")
            return TargetEvaluation(True, "material_capacity_available")

        return TargetEvaluation(False, "unknown_target_kind")

    def select_nearest(
        self,
        targets: Iterable[Target],
        robot_xy: tuple[float, float],
        progress: MissionProgress,
    ) -> Target | None:
        """Select the nearest legal target; confidence breaks equal-distance ties."""
        robot_x, robot_y = robot_xy
        if not math.isfinite(robot_x) or not math.isfinite(robot_y):
            raise ValueError("robot coordinates must be finite")

        eligible: list[tuple[float, float, str, Target]] = []
        for target in targets:
            if self.evaluate(target, progress).eligible:
                distance = math.hypot(target.x - robot_x, target.y - robot_y)
                eligible.append((distance, -target.confidence, target.target_id, target))
        if not eligible:
            return None
        return min(eligible, key=lambda candidate: candidate[:3])[3]
