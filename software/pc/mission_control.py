"""Join STM32 telemetry, guarded odometry, and rescue mission decisions."""

from collections.abc import Sequence
from dataclasses import dataclass

from mission_executor import DeliveryOutcome, PickupOutcome
from mission_runtime import MissionRuntimeResult, RescueMissionRuntime
from odometry import Pose2D
from rescue_vision import Detection
from rover_telemetry import RoverTelemetry, TelemetryStatus
from telemetry_odometry import PoseEstimate, TelemetryOdometry


UINT32_MASK = 0xFFFFFFFF
UINT32_HALF_RANGE = 0x80000000


@dataclass(frozen=True)
class MissionControlConfig:
    battery_stop_mv: int | None = None
    telemetry_max_age_ms: int = 250

    def __post_init__(self) -> None:
        if self.battery_stop_mv is not None and (
            type(self.battery_stop_mv) is not int
            or not 1 <= self.battery_stop_mv <= 0xFFFF
        ):
            raise ValueError("battery_stop_mv must be in 1..65535")
        if (
            type(self.telemetry_max_age_ms) is not int
            or not 1 <= self.telemetry_max_age_ms < UINT32_HALF_RANGE
        ):
            raise ValueError("telemetry_max_age_ms must be below the u32 half range")


@dataclass(frozen=True)
class MissionControlResult:
    mission: MissionRuntimeResult
    pose_estimate: PoseEstimate
    safety_latched: bool
    safety_reason: str | None
    command_confirmation: str


class MissionControlCoordinator:
    """Combine one perception update with the newest available STM32 state.

    The coordinator creates a fail-closed mission command but does not perform
    socket I/O. The caller sends ``result.mission.execution.command`` through
    ``MotionLinkClient`` and supplies PC monotonic timestamps. ToF obstacle
    circles are passed in field coordinates because sensor mount poses are not
    known yet; this class deliberately does not guess their orientation.
    """

    def __init__(
        self,
        runtime: RescueMissionRuntime,
        odometry: TelemetryOdometry,
        config: MissionControlConfig = MissionControlConfig(),
    ) -> None:
        if not isinstance(runtime, RescueMissionRuntime):
            raise TypeError("runtime must be a RescueMissionRuntime")
        if not isinstance(odometry, TelemetryOdometry):
            raise TypeError("odometry must be a TelemetryOdometry")
        if not isinstance(config, MissionControlConfig):
            raise TypeError("config must be a MissionControlConfig")
        self.runtime = runtime
        self.odometry = odometry
        self.config = config
        self._safety_latched = False
        self._safety_reason: str | None = None

    def step(
        self,
        command_sequence: int,
        detections: Sequence[Detection],
        *,
        now_ms: int,
        vision_received_ms: int,
        telemetry: RoverTelemetry | None = None,
        telemetry_received_ms: int | None = None,
        field_obstacles: Sequence[tuple[float, float, float]] = (),
        pickup_outcome: PickupOutcome = PickupOutcome.NONE,
        delivery_outcome: DeliveryOutcome = DeliveryOutcome.NONE,
        emergency_stop: bool = False,
    ) -> MissionControlResult:
        if (telemetry is None) != (telemetry_received_ms is None):
            raise ValueError("telemetry and telemetry_received_ms must be supplied together")
        estimate = (
            self.odometry.update(telemetry, received_at_ms=telemetry_received_ms)
            if telemetry is not None
            else self.odometry.snapshot(now_ms)
        )

        if emergency_stop:
            self._latch_safety("operator_emergency_stop")
        if telemetry is not None:
            if telemetry.status & TelemetryStatus.EMERGENCY_STOP:
                self._latch_safety("stm32_emergency_stop")
            if (
                self.config.battery_stop_mv is not None
                and telemetry.status & TelemetryStatus.BATTERY_VALID
                and telemetry.battery_mv < self.config.battery_stop_mv
            ):
                self._latch_safety("battery_below_configured_threshold")

        mission = self.runtime.step(
            command_sequence,
            detections,
            now_ms=now_ms,
            vision_received_ms=vision_received_ms,
            pose=estimate.pose if estimate.valid else None,
            pose_received_ms=estimate.received_at_ms,
            field_obstacles=field_obstacles,
            pickup_outcome=pickup_outcome,
            delivery_outcome=delivery_outcome,
            emergency_stop=self._safety_latched,
        )
        return MissionControlResult(
            mission=mission,
            pose_estimate=estimate,
            safety_latched=self._safety_latched,
            safety_reason=self._safety_reason,
            command_confirmation=_command_confirmation(
                mission.execution.command.sequence,
                telemetry,
                now_ms=now_ms,
                telemetry_received_ms=telemetry_received_ms,
                max_age_ms=self.config.telemetry_max_age_ms,
            ),
        )

    def clear_safety_latch(self, *, operator_confirmed: bool) -> None:
        """Clear only after an explicit operator action and physical inspection."""
        if not operator_confirmed:
            raise ValueError("operator confirmation is required to clear the safety latch")
        self._safety_latched = False
        self._safety_reason = None

    def _latch_safety(self, reason: str) -> None:
        self._safety_latched = True
        self._safety_reason = reason


def _command_confirmation(
    sequence: int,
    telemetry: RoverTelemetry | None,
    *,
    now_ms: int,
    telemetry_received_ms: int | None,
    max_age_ms: int,
) -> str:
    if (
        telemetry is None
        or telemetry_received_ms is None
        or not telemetry.status & TelemetryStatus.LAST_COMMAND_VALID
        or not _is_fresh_u32(now_ms, telemetry_received_ms, max_age_ms)
    ):
        return "not_reported"
    if telemetry.last_command_sequence == sequence:
        return "accepted"
    delta = (telemetry.last_command_sequence - sequence) & 0xFFFFFFFF
    if 0 < delta < 0x80000000:
        return "a_newer_command_was_accepted"
    return "awaiting_acceptance"


def _is_fresh_u32(now_ms: int, received_ms: int, max_age_ms: int) -> bool:
    if (
        type(now_ms) is not int
        or type(received_ms) is not int
        or not 0 <= now_ms <= UINT32_MASK
        or not 0 <= received_ms <= UINT32_MASK
    ):
        return False
    age = (now_ms - received_ms) & UINT32_MASK
    return age < UINT32_HALF_RANGE and age <= max_age_ms
