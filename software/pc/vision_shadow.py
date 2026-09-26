"""Safe perception/planning observation and deterministic mission-cycle replay.

This module records detections and planner outputs for inspection. It never
opens a motion link or transmits a command to the rover.
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from typing import Callable

from autonomy_cycle import AutonomyCycle, AutonomyCycleConfig
from grid_planner import GridMap
from mission_policy import MissionProgress, TargetKind
from odometry import Pose2D
from path_follower import FollowerConfig
from pixel_ground import GroundPlaneProjector
from rescue_transport import ImageFrame
from rescue_vision import Detection


LOG_SCHEMA = "rescue-shadow-v1"


@dataclass(frozen=True)
class ReplayReport:
    cycles: int
    matched: int
    mismatched_lines: tuple[int, ...]


class ShadowLogWriter:
    """Write append-only JSONL observations, refusing to overwrite a prior run."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = self.path.open("x", encoding="utf-8", newline="\n")
        self._write(
            {
                "schema": LOG_SCHEMA,
                "event": "session",
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "actuation_enabled": False,
            }
        )

    def record_vision_frame(
        self,
        frame: ImageFrame,
        detections: list[Detection],
        *,
        image_size: tuple[int, int],
        received_monotonic_ns: int,
        inference_ms: float | None,
        detector_enabled: bool,
    ) -> None:
        width, height = image_size
        if width <= 0 or height <= 0:
            raise ValueError("image dimensions must be positive")
        if type(received_monotonic_ns) is not int or received_monotonic_ns < 0:
            raise ValueError("received_monotonic_ns must be a nonnegative integer")
        if inference_ms is not None and (not math.isfinite(inference_ms) or inference_ms < 0):
            raise ValueError("inference_ms must be finite and nonnegative")
        self._write(
            {
                "schema": LOG_SCHEMA,
                "event": "vision_frame",
                "frame": {
                    "sequence": frame.seq,
                    "camera_ticks_ms": frame.camera_ticks_ms,
                    "width": width,
                    "height": height,
                    "received_monotonic_ns": received_monotonic_ns,
                },
                "detector_enabled": bool(detector_enabled),
                "inference_ms": inference_ms,
                "detections": [_detection_to_json(item) for item in detections],
                "mission_status": "not_evaluated_without_pose_and_calibration",
                "actuation_sent": False,
            }
        )

    def record_cycle(
        self,
        *,
        frame_sequence: int,
        command_sequence: int,
        detections: list[Detection],
        now_ms: int,
        vision_received_ms: int,
        pose: Pose2D | None,
        pose_received_ms: int | None,
        progress: MissionProgress,
        emergency_stop: bool,
        result,
    ) -> None:
        self._write(
            {
                "schema": LOG_SCHEMA,
                "event": "mission_cycle",
                "frame_sequence": frame_sequence,
                "input": {
                    "command_sequence": command_sequence,
                    "detections": [_detection_to_json(item) for item in detections],
                    "now_ms": now_ms,
                    "vision_received_ms": vision_received_ms,
                    "pose": None if pose is None else asdict(pose),
                    "pose_received_ms": pose_received_ms,
                    "progress": asdict(progress),
                    "emergency_stop": emergency_stop,
                },
                "output": _cycle_result_to_json(result),
                "actuation_sent": False,
            }
        )

    def _write(self, record: dict) -> None:
        json.dump(record, self._stream, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        self._stream.write("\n")
        self._stream.flush()

    def close(self) -> None:
        if not self._stream.closed:
            self._stream.close()

    def __enter__(self) -> "ShadowLogWriter":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


class ShadowCycleRunner:
    """Run the normal PC decision cycle and log its output without actuating."""

    def __init__(self, cycle: AutonomyCycle, log: ShadowLogWriter):
        if not isinstance(cycle, AutonomyCycle):
            raise TypeError("cycle must be an AutonomyCycle")
        if not isinstance(log, ShadowLogWriter):
            raise TypeError("log must be a ShadowLogWriter")
        self.cycle = cycle
        self.log = log

    def step(
        self,
        *,
        frame_sequence: int,
        command_sequence: int,
        detections: list[Detection],
        now_ms: int,
        vision_received_ms: int,
        pose: Pose2D | None,
        pose_received_ms: int | None,
        progress: MissionProgress,
        emergency_stop: bool = False,
    ):
        result = self.cycle.run(
            command_sequence,
            detections,
            now_ms=now_ms,
            vision_received_ms=vision_received_ms,
            pose=pose,
            pose_received_ms=pose_received_ms,
            progress=progress,
            emergency_stop=emergency_stop,
        )
        self.log.record_cycle(
            frame_sequence=frame_sequence,
            command_sequence=command_sequence,
            detections=detections,
            now_ms=now_ms,
            vision_received_ms=vision_received_ms,
            pose=pose,
            pose_received_ms=pose_received_ms,
            progress=progress,
            emergency_stop=emergency_stop,
            result=result,
        )
        return result


def replay_cycle_log(path: Path, cycle_factory: Callable[[], AutonomyCycle]) -> ReplayReport:
    """Re-run logged mission-cycle inputs and compare their deterministic outputs."""
    cycle = cycle_factory()
    if not isinstance(cycle, AutonomyCycle):
        raise TypeError("cycle_factory must return an AutonomyCycle")
    cycles = 0
    mismatches: list[int] = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"invalid JSON on line {line_number}") from error
            if record.get("schema") != LOG_SCHEMA:
                raise ValueError(f"unsupported shadow log schema on line {line_number}")
            if record.get("event") != "mission_cycle":
                continue
            if record.get("actuation_sent") is not False:
                raise ValueError(f"shadow log contains an actuated cycle on line {line_number}")
            values = record.get("input")
            if not isinstance(values, dict):
                raise ValueError(f"missing cycle input on line {line_number}")
            detections = [_detection_from_json(value) for value in values["detections"]]
            pose_value = values["pose"]
            pose = None if pose_value is None else Pose2D(**pose_value)
            result = cycle.run(
                values["command_sequence"],
                detections,
                now_ms=values["now_ms"],
                vision_received_ms=values["vision_received_ms"],
                pose=pose,
                pose_received_ms=values["pose_received_ms"],
                progress=MissionProgress(**values["progress"]),
                emergency_stop=values["emergency_stop"],
            )
            cycles += 1
            if _cycle_result_to_json(result) != record.get("output"):
                mismatches.append(line_number)
    return ReplayReport(cycles, cycles - len(mismatches), tuple(mismatches))


def create_demo_cycle() -> AutonomyCycle:
    """Create an explicitly synthetic cycle for software-path checks only."""
    grid = GridMap(2.0, 2.0, 0.05)
    projector = GroundPlaneProjector.from_correspondences(
        [(0, 0), (100, 0), (100, 100), (0, 100)],
        [(0, 0), (1, 0), (1, 1), (0, 1)],
        image_size=(100, 100),
    )
    config = AutonomyCycleConfig(
        class_kind_by_id={
            0: TargetKind.ORDINARY,
            1: TargetKind.CORE,
            2: TargetKind.CASUALTY,
            3: TargetKind.DANGER,
            4: TargetKind.OPPONENT_ROBOT,
        },
        obstacle_radius_m_by_class={3: 0.08, 4: 0.12},
        unknown_obstacle_radius_m=0.06,
        obstacle_inflation_m=0.10,
        vision_max_age_ms=250,
        pose_max_age_ms=100,
        command_lease_ms=150,
        track_width_m=0.20,
        max_wheel_speed_m_s=0.40,
        target_standoff_m=0.06,
    )
    return AutonomyCycle(grid, projector, FollowerConfig(0.15, 0.20, 1.0), config)


def record_demo(path: Path) -> int:
    """Record synthetic detections, including safe-stop cases, for replay."""
    cycle = create_demo_cycle()
    robot_pose = Pose2D(0.25, 0.25, 0.0)
    cases = (
        (1, [Detection(0, "ordinary_material", 0.96, (35, 35, 45, 45))], 1_000, robot_pose, 950),
        (
            2,
            [
                Detection(0, "ordinary_material", 0.94, (80, 35, 90, 45)),
                Detection(3, "danger_object", 0.88, (50, 35, 60, 45)),
            ],
            1_050,
            robot_pose,
            1_025,
        ),
        (3, [], 1_100, robot_pose, 1_075),
        (4, [Detection(0, "ordinary_material", 0.90, (35, 35, 45, 45))], 1_400, robot_pose, 1_390),
        (5, [Detection(0, "ordinary_material", 0.90, (35, 35, 45, 45))], 1_150, None, None),
    )
    with ShadowLogWriter(path) as log:
        runner = ShadowCycleRunner(cycle, log)
        for sequence, detections, now_ms, pose, pose_ms in cases:
            runner.step(
                frame_sequence=sequence,
                command_sequence=sequence,
                detections=detections,
                now_ms=now_ms,
                vision_received_ms=max(0, now_ms - 300) if sequence == 4 else max(0, now_ms - 25),
                pose=pose,
                pose_received_ms=pose_ms,
                progress=MissionProgress(),
            )
    return len(cases)


def _detection_to_json(detection: Detection) -> dict:
    return {
        "class_id": detection.class_id,
        "label": detection.label,
        "confidence": detection.confidence,
        "xyxy": list(detection.xyxy),
    }


def _detection_from_json(value: dict) -> Detection:
    return Detection(
        class_id=int(value["class_id"]),
        label=str(value["label"]),
        confidence=float(value["confidence"]),
        xyxy=tuple(int(coordinate) for coordinate in value["xyxy"]),
    )


def _cycle_result_to_json(result) -> dict:
    command = result.motion_command
    target = result.target
    route = result.route
    return {
        "status": result.status,
        "motion_command": {
            "sequence": command.sequence,
            "lease_ms": command.lease_ms,
            "left_mm_s": command.left_mm_s,
            "right_mm_s": command.right_mm_s,
            "actuator": int(command.actuator),
        },
        "motion_frame_hex": result.frame.hex(),
        "velocity": {
            "linear_m_s": result.velocity.linear_m_s,
            "angular_rad_s": result.velocity.angular_rad_s,
            "done": result.velocity.done,
        },
        "target": None
        if target is None
        else {
            "target_id": target.target_id,
            "kind": target.kind.value,
            "x": target.x,
            "y": target.y,
            "confidence": target.confidence,
        },
        "route": None
        if route is None
        else {
            "waypoints_m": [list(point) for point in route.waypoints_m],
            "route_length_m": route.route_length_m,
        },
        "projected_target_count": result.projected_target_count,
        "dynamic_obstacle_count": result.dynamic_obstacle_count,
        "actuation_sent": False,
    }


def _main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    demo = commands.add_parser("demo", help="record a synthetic shadow-cycle log")
    demo.add_argument("--log", type=Path, required=True)
    replay = commands.add_parser("replay", help="re-run and compare a shadow-cycle log")
    replay.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()

    if args.command == "demo":
        count = record_demo(args.log)
        print(f"Recorded {count} synthetic decision cycles to {args.log}")
        print("All motion commands were logged only; no hardware link was opened.")
        return 0

    report = replay_cycle_log(args.log, create_demo_cycle)
    print(
        f"Replayed {report.cycles} cycles: {report.matched} matched, "
        f"{len(report.mismatched_lines)} mismatched."
    )
    if report.mismatched_lines:
        print(f"Mismatched log lines: {', '.join(map(str, report.mismatched_lines))}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
