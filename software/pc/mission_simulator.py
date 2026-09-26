"""Deterministic no-hardware mission simulation and JSONL frame replay.

Run from this directory with ``python mission_simulator.py``. The simulated
pickup/drop confirmations and odometry are idealized; this is for checking
software flow and rule transitions, not estimating real-robot performance.
"""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
from collections.abc import Iterable

from grid_planner import GridMap
from mission_executor import (
    DeliveryOutcome,
    MissionExecutionResult,
    MissionExecutor,
    MissionExecutorConfig,
    MissionState,
    PickupOutcome,
)
from mission_policy import MissionProgress, Target, TargetKind
from odometry import Pose2D
from path_follower import FollowerConfig


STEP_MS = 50
FIELD_SIZE_M = 3.0
MATERIAL_ZONE = (0.30, 0.30)
CASUALTY_ZONE = (0.30, 0.55)
SCENARIO_TARGETS = (
    Target("first-ordinary", TargetKind.ORDINARY, 0.95, 0.45, 0.98),
    Target("batch-ordinary-a", TargetKind.ORDINARY, 2.45, 0.45, 0.97),
    Target("batch-core", TargetKind.CORE, 2.40, 1.25, 0.96),
    Target("batch-ordinary-b", TargetKind.ORDINARY, 2.40, 2.45, 0.95),
    Target("casualty", TargetKind.CASUALTY, 2.55, 2.45, 0.99),
)
HAZARD_CIRCLES = ((1.45, 0.80, 0.16),)


def create_executor() -> MissionExecutor:
    """Create the documented synthetic 3 m square field scenario."""
    grid = GridMap(FIELD_SIZE_M, FIELD_SIZE_M, 0.05)
    grid.add_rect_obstacle(1.25, 1.10, 1.55, 1.45)
    config = MissionExecutorConfig(
        material_zone_xy=MATERIAL_ZONE,
        casualty_zone_xy=CASUALTY_ZONE,
        pickup_standoff_m=0.09,
        zone_tolerance_m=0.10,
        obstacle_inflation_m=0.12,
        vision_max_age_ms=300,
        pose_max_age_ms=150,
        command_lease_ms=150,
        track_width_m=0.20,
        max_wheel_speed_m_s=0.30,
    )
    follower = FollowerConfig(
        lookahead_m=0.16,
        linear_speed_m_s=0.20,
        max_angular_speed_rad_s=1.2,
        goal_tolerance_m=0.08,
        rotate_in_place_threshold_rad=math.radians(8.0),
    )
    return MissionExecutor(grid, follower, config)


def targets_for_progress(
    progress: MissionProgress,
    available_ids: set[str],
) -> tuple[Target, ...]:
    if not progress.first_ordinary_delivered:
        phase_ids = {"first-ordinary"}
    elif progress.delivered_material_count < 4:
        phase_ids = {"batch-ordinary-a", "batch-core", "batch-ordinary-b"}
    elif progress.delivered_casualty_count == 0:
        phase_ids = {"casualty"}
    else:
        phase_ids = set()
    return tuple(
        target
        for target in SCENARIO_TARGETS
        if target.target_id in phase_ids and target.target_id in available_ids
    )


def _target_to_json(target: Target) -> dict[str, object]:
    return {
        "target_id": target.target_id,
        "kind": target.kind.value,
        "x": target.x,
        "y": target.y,
        "confidence": target.confidence,
    }


def _target_from_json(value: dict[str, object]) -> Target:
    return Target(
        str(value["target_id"]),
        TargetKind(str(value["kind"])),
        float(value["x"]),
        float(value["y"]),
        float(value["confidence"]),
    )


def _progress_to_json(progress: MissionProgress) -> dict[str, object]:
    return asdict(progress)


def _record_row(
    result: MissionExecutionResult,
    *,
    sequence: int,
    now_ms: int,
    vision_received_ms: int,
    pose_received_ms: int,
    pose: Pose2D,
    targets: Iterable[Target],
    pickup_outcome: PickupOutcome,
    delivery_outcome: DeliveryOutcome,
) -> dict[str, object]:
    return {
        "input": {
            "sequence": sequence,
            "now_ms": now_ms,
            "vision_received_ms": vision_received_ms,
            "pose_received_ms": pose_received_ms,
            "pose": {"x": pose.x, "y": pose.y, "yaw": pose.yaw},
            "targets": [_target_to_json(target) for target in targets],
            "obstacles": HAZARD_CIRCLES,
            "pickup_outcome": pickup_outcome.value,
            "delivery_outcome": delivery_outcome.value,
        },
        "expected": {
            "state": result.state.value,
            "status": result.status,
            "frame_hex": result.frame.hex(),
            "progress": _progress_to_json(result.progress),
        },
    }


def simulate(
    *,
    max_seconds: float = 180.0,
    step_ms: int = STEP_MS,
    camera_lag_ms: int = 0,
    odometry_lag_ms: int = 0,
    drop_vision_every: int = 0,
    drop_odometry_every: int = 0,
    drop_command_every: int = 0,
    record_path: Path | None = None,
) -> int:
    """Run an ideal-kinematics scenario and optionally save every input/output."""
    if not math.isfinite(max_seconds) or max_seconds <= 0:
        raise ValueError("max_seconds must be positive and finite")
    if type(step_ms) is not int or step_ms < 10:
        raise ValueError("step_ms must be an integer of at least 10 ms")
    if min(
        camera_lag_ms,
        odometry_lag_ms,
        drop_vision_every,
        drop_odometry_every,
        drop_command_every,
    ) < 0:
        raise ValueError("lag and drop options cannot be negative")

    executor = create_executor()
    available_ids = {target.target_id for target in SCENARIO_TARGETS}
    pose = Pose2D(*MATERIAL_ZONE, 0.0)
    seq = 0
    now_ms = 0
    max_steps = math.ceil(max_seconds * 1000.0 / step_ms)
    pending_pickup = PickupOutcome.NONE
    pending_pickup_id: str | None = None
    pending_delivery = DeliveryOutcome.NONE
    previous_result: MissionExecutionResult | None = None
    last_applied = (0, 0)
    last_command_ms = 0
    applied_commands = 0
    dropped_commands = 0
    record_file = None

    if record_path is not None:
        record_path.parent.mkdir(parents=True, exist_ok=True)
        record_file = record_path.open("w", encoding="utf-8", newline="\n")

    try:
        for _ in range(max_steps):
            if pending_pickup is PickupOutcome.CONFIRMED and pending_pickup_id:
                available_ids.discard(pending_pickup_id)
            visible_targets = targets_for_progress(executor.progress, available_ids)
            vision_stamp = (now_ms - camera_lag_ms) & 0xFFFFFFFF
            pose_stamp = (now_ms - odometry_lag_ms) & 0xFFFFFFFF
            if camera_lag_ms > 300 or (
                drop_vision_every and (seq + 1) % drop_vision_every == 0
            ):
                vision_stamp = (now_ms - 301) & 0xFFFFFFFF
            if odometry_lag_ms > 150 or (
                drop_odometry_every and (seq + 1) % drop_odometry_every == 0
            ):
                pose_stamp = (now_ms - 151) & 0xFFFFFFFF

            result = executor.step(
                seq,
                visible_targets,
                now_ms=now_ms,
                vision_received_ms=vision_stamp,
                pose=pose,
                pose_received_ms=pose_stamp,
                obstacles=HAZARD_CIRCLES,
                pickup_outcome=pending_pickup,
                delivery_outcome=pending_delivery,
            )
            if record_file is not None:
                row = _record_row(
                    result,
                    sequence=seq,
                    now_ms=now_ms,
                    vision_received_ms=vision_stamp,
                    pose_received_ms=pose_stamp,
                    pose=pose,
                    targets=visible_targets,
                    pickup_outcome=pending_pickup,
                    delivery_outcome=pending_delivery,
                )
                record_file.write(json.dumps(row, separators=(",", ":")) + "\n")

            if pending_pickup is PickupOutcome.CONFIRMED:
                pending_pickup_id = None
            pending_pickup = PickupOutcome.NONE
            pending_delivery = DeliveryOutcome.NONE
            if result.state is MissionState.ACQUIRE:
                pending_pickup = PickupOutcome.CONFIRMED
                if result.active_target is not None:
                    pending_pickup_id = result.active_target.target_id
            elif result.state is MissionState.DELIVER:
                pending_delivery = DeliveryOutcome.CONFIRMED

            command_dropped = bool(
                drop_command_every and (seq + 1) % drop_command_every == 0
            )
            if command_dropped:
                dropped_commands += 1
                if now_ms - last_command_ms >= executor.config.command_lease_ms:
                    last_applied = (0, 0)
            else:
                last_applied = (
                    result.command.left_mm_s,
                    result.command.right_mm_s,
                )
                last_command_ms = now_ms
                applied_commands += 1

            left_m_s = last_applied[0] / 1000.0
            right_m_s = last_applied[1] / 1000.0
            linear = (left_m_s + right_m_s) / 2.0
            angular = (right_m_s - left_m_s) / executor.config.track_width_m
            half_turn = pose.yaw + angular * (step_ms / 1000.0) / 2.0
            pose = Pose2D(
                pose.x + linear * math.cos(half_turn) * (step_ms / 1000.0),
                pose.y + linear * math.sin(half_turn) * (step_ms / 1000.0),
                (pose.yaw + angular * (step_ms / 1000.0) + math.pi)
                % (2.0 * math.pi)
                - math.pi,
            )
            previous_result = result
            seq = (seq + 1) & 0xFFFFFFFF
            now_ms = (now_ms + step_ms) & 0xFFFFFFFF

            if (
                executor.progress.delivered_material_count >= 4
                and executor.progress.delivered_casualty_count >= 1
                and not executor.progress.carried_material_count
                and not executor.progress.casualty_onboard
            ):
                elapsed_s = (seq * step_ms) / 1000.0
                print(
                    "SIMULATION_COMPLETE "
                    f"elapsed_s={elapsed_s:.1f} "
                    f"steps={seq} "
                    f"materials={executor.progress.delivered_material_count} "
                    f"casualties={executor.progress.delivered_casualty_count} "
                    f"commands_applied={applied_commands} "
                    f"commands_dropped={dropped_commands} "
                    f"pose=({pose.x:.2f},{pose.y:.2f})"
                )
                return 0

        final = previous_result
        print(
            "SIMULATION_TIMEOUT "
            f"elapsed_s={max_seconds:.1f} "
            f"state={final.state.value if final else 'not_started'} "
            f"status={final.status if final else 'not_started'} "
            f"materials={executor.progress.delivered_material_count} "
            f"casualties={executor.progress.delivered_casualty_count} "
            f"carried_materials={executor.progress.carried_material_count} "
            f"casualty_onboard={executor.progress.casualty_onboard} "
            f"commands_applied={applied_commands} "
            f"commands_dropped={dropped_commands}"
        )
        return 1
    finally:
        if record_file is not None:
            record_file.close()


def replay(record_path: Path) -> int:
    """Re-run recorded executor inputs and compare every command and state."""
    executor = create_executor()
    frames = 0
    with record_path.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            item = row["input"]
            pose_data = item["pose"]
            result = executor.step(
                int(item["sequence"]),
                tuple(_target_from_json(target) for target in item["targets"]),
                now_ms=int(item["now_ms"]),
                vision_received_ms=int(item["vision_received_ms"]),
                pose=Pose2D(
                    float(pose_data["x"]),
                    float(pose_data["y"]),
                    float(pose_data["yaw"]),
                ),
                pose_received_ms=int(item["pose_received_ms"]),
                obstacles=tuple(tuple(obstacle) for obstacle in item["obstacles"]),
                pickup_outcome=PickupOutcome(item["pickup_outcome"]),
                delivery_outcome=DeliveryOutcome(item["delivery_outcome"]),
            )
            expected = row["expected"]
            actual = {
                "state": result.state.value,
                "status": result.status,
                "frame_hex": result.frame.hex(),
                "progress": _progress_to_json(result.progress),
            }
            if actual != expected:
                print(
                    f"REPLAY_MISMATCH line={line_number} "
                    f"expected={expected} actual={actual}"
                )
                return 1
            frames += 1
    print(f"REPLAY_OK frames={frames} source={record_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, help="write a JSONL run log")
    parser.add_argument("--replay", type=Path, help="replay and compare a JSONL run log")
    parser.add_argument("--max-seconds", type=float, default=180.0)
    parser.add_argument("--step-ms", type=int, default=STEP_MS)
    parser.add_argument("--camera-lag-ms", type=int, default=0)
    parser.add_argument("--odometry-lag-ms", type=int, default=0)
    parser.add_argument(
        "--drop-vision-every",
        type=int,
        default=0,
        help="make every Nth camera frame stale; 0 disables vision loss",
    )
    parser.add_argument(
        "--drop-odometry-every",
        type=int,
        default=0,
        help="make every Nth pose stale; 0 disables pose loss",
    )
    parser.add_argument(
        "--drop-command-every",
        type=int,
        default=0,
        help="drop every Nth motion frame; 0 disables command loss",
    )
    args = parser.parse_args()
    if args.replay:
        return replay(args.replay)
    return simulate(
        max_seconds=args.max_seconds,
        step_ms=args.step_ms,
        camera_lag_ms=args.camera_lag_ms,
        odometry_lag_ms=args.odometry_lag_ms,
        drop_vision_every=args.drop_vision_every,
        drop_odometry_every=args.drop_odometry_every,
        drop_command_every=args.drop_command_every,
        record_path=args.record,
    )


if __name__ == "__main__":
    raise SystemExit(main())
