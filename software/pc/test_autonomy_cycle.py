import unittest

from autonomy_cycle import AutonomyCycle, AutonomyCycleConfig
from grid_planner import GridMap
from mission_policy import MissionProgress, TargetKind
from motion_protocol import ActuatorCommand, MotionCommandDecoder
from odometry import Pose2D
from path_follower import FollowerConfig
from pixel_ground import GroundPlaneProjector
from rescue_vision import Detection


class AutonomyCycleTests(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap(2.0, 2.0, 0.05)
        self.projector = GroundPlaneProjector.from_correspondences(
            [(0, 0), (100, 0), (100, 100), (0, 100)],
            [(0, 0), (1, 0), (1, 1), (0, 1)],
            image_size=(100, 100),
        )
        self.config = AutonomyCycleConfig(
            class_kind_by_id={0: TargetKind.ORDINARY, 1: TargetKind.DANGER},
            obstacle_radius_m_by_class={1: 0.08},
            unknown_obstacle_radius_m=0.06,
            obstacle_inflation_m=0.10,
            vision_max_age_ms=250,
            pose_max_age_ms=100,
            command_lease_ms=150,
            track_width_m=0.20,
            max_wheel_speed_m_s=0.40,
            target_standoff_m=0.06,
        )
        self.cycle = AutonomyCycle(
            self.grid,
            self.projector,
            FollowerConfig(0.15, 0.20, 1.0),
            self.config,
        )
        self.detection = Detection(0, "ordinary", 0.95, (35, 35, 45, 45))

    def run_cycle(self, **overrides):
        args = {
            "command_sequence": 20,
            "detections": [self.detection],
            "now_ms": 1_000,
            "vision_received_ms": 900,
            "pose": Pose2D(0.25, 0.25, 0.0),
            "pose_received_ms": 950,
            "progress": MissionProgress(),
        }
        args.update(overrides)
        return self.cycle.run(**args)

    def test_fresh_inputs_flow_from_detection_to_wire_command(self):
        result = self.run_cycle()
        self.assertEqual(result.status, "tracking")
        self.assertIsNotNone(result.target)
        self.assertEqual(result.target.kind, TargetKind.ORDINARY)
        self.assertEqual(result.projected_target_count, 1)
        self.assertNotEqual(
            (result.motion_command.left_mm_s, result.motion_command.right_mm_s),
            (0, 0),
        )
        decoded = MotionCommandDecoder().feed(result.frame)
        self.assertEqual(decoded, [result.motion_command])
        self.assertEqual(result.motion_command.actuator, ActuatorCommand.HOLD)

    def test_stale_vision_or_pose_emits_zero_speed_frame(self):
        for overrides, expected in (
            ({"vision_received_ms": 600}, "vision_stale"),
            ({"pose_received_ms": 800}, "pose_stale_or_invalid"),
            ({"pose": None}, "pose_stale_or_invalid"),
        ):
            result = self.run_cycle(**overrides)
            self.assertEqual(result.status, expected)
            self.assertEqual(
                (result.motion_command.left_mm_s, result.motion_command.right_mm_s),
                (0, 0),
            )

    def test_emergency_stop_has_priority_and_wrap_safe_freshness(self):
        result = self.run_cycle(emergency_stop=True, vision_received_ms=1)
        self.assertEqual(result.status, "emergency_stop")
        self.assertEqual(result.motion_command.left_mm_s, 0)
        wrapped = self.run_cycle(now_ms=20, vision_received_ms=0xFFFFFFF0, pose_received_ms=0xFFFFFFF0)
        self.assertEqual(wrapped.status, "tracking")

    def test_hazards_and_unknown_classes_become_dynamic_obstacles(self):
        result = self.run_cycle(
            detections=[
                self.detection,
                Detection(1, "danger", 0.80, (15, 15, 25, 25)),
                Detection(99, "unmapped", 0.90, (20, 20, 30, 30)),
            ]
        )
        self.assertEqual(result.status, "tracking")
        self.assertEqual(result.dynamic_obstacle_count, 2)
        self.assertEqual(result.projected_target_count, 2)
        straight_line_distance = ((0.65 - 0.25) ** 2 + (0.70 - 0.25) ** 2) ** 0.5
        self.assertGreater(result.route.route_length_m, straight_line_distance + 0.05)
        # Dynamic obstacle insertion must not mutate the reusable static map.
        self.assertFalse(self.grid.is_blocked(self.grid.world_to_cell(0.70, 0.70)))

    def test_no_legal_target_and_missing_calibration_stop(self):
        casualty_only = Detection(0, "ordinary", 0.95, (35, 35, 45, 45))
        result = self.run_cycle(
            detections=[casualty_only],
            progress=MissionProgress(
                first_ordinary_delivered=True,
                delivered_material_count=3,
                carried_material_count=3,
            ),
        )
        self.assertEqual(result.status, "no_legal_reachable_target")
        self.assertEqual(result.motion_command.left_mm_s, 0)

        uncalibrated = AutonomyCycle(self.grid, None, FollowerConfig(0.15, 0.20, 1.0), self.config)
        result = uncalibrated.run(
            21,
            [self.detection],
            now_ms=1_000,
            vision_received_ms=900,
            pose=Pose2D(0.25, 0.25, 0.0),
            pose_received_ms=950,
            progress=MissionProgress(),
        )
        self.assertEqual(result.status, "ground_calibration_missing")
        self.assertEqual(result.motion_command.right_mm_s, 0)

    def test_pose_outside_map_fails_closed(self):
        result = self.run_cycle(pose=Pose2D(3.0, 3.0, 0.0))
        self.assertEqual(result.status, "pose_outside_map")
        self.assertEqual(
            (result.motion_command.left_mm_s, result.motion_command.right_mm_s),
            (0, 0),
        )

    def test_invalid_projection_and_config_are_rejected_or_fail_closed(self):
        result = self.run_cycle(detections=[Detection(0, "ordinary", 0.9, (110, 10, 120, 20))])
        self.assertEqual(result.status, "invalid_detection_geometry")
        self.assertEqual(result.motion_command.left_mm_s, 0)
        with self.assertRaises(ValueError):
            AutonomyCycleConfig(
                **{
                    **self.config.__dict__,
                    "class_kind_by_id": {1: TargetKind.DANGER},
                    "obstacle_radius_m_by_class": {},
                }
            )


if __name__ == "__main__":
    unittest.main()
