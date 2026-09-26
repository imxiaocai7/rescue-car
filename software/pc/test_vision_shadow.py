import json
from pathlib import Path
import tempfile
import unittest

from mission_policy import MissionProgress
from rescue_transport import ImageFrame
from rescue_vision import Detection
from vision_shadow import (
    ShadowCycleRunner,
    ShadowLogWriter,
    create_demo_cycle,
    record_demo,
    replay_cycle_log,
)


class VisionShadowTests(unittest.TestCase):
    def test_synthetic_cycle_log_replays_exactly_and_never_actuates(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "demo.jsonl"
            self.assertEqual(record_demo(log_path), 5)

            report = replay_cycle_log(log_path, create_demo_cycle)
            self.assertEqual((report.cycles, report.matched, report.mismatched_lines), (5, 5, ()))

            records = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
            cycles = [record for record in records if record["event"] == "mission_cycle"]
            self.assertEqual(
                [record["output"]["status"] for record in cycles],
                [
                    "tracking",
                    "tracking",
                    "no_legal_reachable_target",
                    "vision_stale",
                    "pose_stale_or_invalid",
                ],
            )
            self.assertTrue(all(record["actuation_sent"] is False for record in cycles))

    def test_mission_cycle_logs_fail_closed_output_without_motion_link(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "cycle.jsonl"
            with ShadowLogWriter(log_path) as log:
                runner = ShadowCycleRunner(create_demo_cycle(), log)
                result = runner.step(
                    frame_sequence=9,
                    command_sequence=9,
                    detections=[Detection(0, "ordinary_material", 0.9, (35, 35, 45, 45))],
                    now_ms=1_000,
                    vision_received_ms=990,
                    pose=None,
                    pose_received_ms=None,
                    progress=MissionProgress(),
                )

            self.assertEqual(result.status, "pose_stale_or_invalid")
            record = json.loads(log_path.read_text(encoding="utf-8").splitlines()[1])
            self.assertFalse(record["actuation_sent"])
            self.assertEqual(record["output"]["motion_command"]["left_mm_s"], 0)
            self.assertEqual(record["output"]["motion_command"]["right_mm_s"], 0)

    def test_vision_frame_log_records_detector_output_and_timing(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "vision.jsonl"
            frame = ImageFrame(seq=4, camera_ticks_ms=321, width=320, height=240, jpeg=b"")
            detection = Detection(2, "casualty", 0.91, (10, 20, 40, 60))
            with ShadowLogWriter(log_path) as log:
                log.record_vision_frame(
                    frame,
                    [detection],
                    image_size=(320, 240),
                    received_monotonic_ns=123_456,
                    inference_ms=12.5,
                    detector_enabled=True,
                )

            records = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(records[1]["event"], "vision_frame")
            self.assertEqual(records[1]["frame"]["sequence"], 4)
            self.assertEqual(records[1]["detections"][0]["xyxy"], [10, 20, 40, 60])
            self.assertEqual(records[1]["inference_ms"], 12.5)
            self.assertFalse(records[1]["actuation_sent"])

    def test_replay_detects_modified_output_and_refuses_claimed_actuation(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "demo.jsonl"
            record_demo(log_path)
            lines = log_path.read_text(encoding="utf-8").splitlines()
            cycle_record = json.loads(lines[1])
            cycle_record["output"]["status"] = "modified"
            lines[1] = json.dumps(cycle_record)
            log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            report = replay_cycle_log(log_path, create_demo_cycle)
            self.assertEqual(report.mismatched_lines, (2,))

            cycle_record["actuation_sent"] = True
            lines[1] = json.dumps(cycle_record)
            log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "actuated cycle"):
                replay_cycle_log(log_path, create_demo_cycle)

    def test_writer_refuses_to_overwrite_existing_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            log_path = Path(temporary) / "existing.jsonl"
            log_path.write_text("preserve me", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                ShadowLogWriter(log_path)


if __name__ == "__main__":
    unittest.main()
