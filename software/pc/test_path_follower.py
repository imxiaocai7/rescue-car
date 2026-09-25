import math
import unittest

from odometry import Pose2D
from path_follower import (
    FollowerConfig,
    PurePursuitFollower,
    VelocityCommand,
    differential_wheel_speeds,
)


class PathFollowerTests(unittest.TestCase):
    def follower(self, **overrides):
        config = {
            "lookahead_m": 0.25,
            "linear_speed_m_s": 0.2,
            "max_angular_speed_rad_s": 1.0,
            "goal_tolerance_m": 0.05,
        }
        config.update(overrides)
        return PurePursuitFollower(FollowerConfig(**config))

    def test_empty_path_completes_safely(self):
        command = self.follower().update(Pose2D())
        self.assertEqual(command, VelocityCommand(0.0, 0.0, done=True))

    def test_straight_path_produces_forward_only_command(self):
        follower = self.follower()
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        command = follower.update(Pose2D())
        self.assertAlmostEqual(command.linear_m_s, 0.2)
        self.assertAlmostEqual(command.angular_rad_s, 0.0)
        self.assertFalse(command.done)

    def test_bending_path_turns_toward_positive_y(self):
        follower = self.follower(lookahead_m=0.6)
        follower.set_path([(0.0, 0.0), (0.5, 0.5), (0.7, 0.8)])
        command = follower.update(Pose2D())
        self.assertGreater(command.angular_rad_s, 0.0)
        self.assertLessEqual(abs(command.angular_rad_s), 1.0)

    def test_lookahead_does_not_preview_across_a_corner(self):
        follower = self.follower(lookahead_m=0.75)
        follower.set_path([(0.0, 0.0), (0.5, 0.0), (0.5, 0.5)])
        command = follower.update(Pose2D())
        self.assertAlmostEqual(command.angular_rad_s, 0.0)
        self.assertGreater(command.linear_m_s, 0.0)

    def test_waypoint_behind_causes_in_place_turn(self):
        follower = self.follower()
        follower.set_path([(0.0, 0.0), (-1.0, 0.0)])
        command = follower.update(Pose2D())
        self.assertEqual(command.linear_m_s, 0.0)
        # At exactly 180 degrees either rotation direction is equally short.
        self.assertAlmostEqual(abs(command.angular_rad_s), 1.0)

    def test_large_heading_error_rotates_before_translating(self):
        follower = self.follower()
        follower.set_path([(0.0, 0.0), (0.0, 1.0)])
        command = follower.update(Pose2D())
        self.assertEqual(command.linear_m_s, 0.0)
        self.assertGreater(command.angular_rad_s, 0.0)

    def test_goal_tolerance_stops(self):
        follower = self.follower(goal_tolerance_m=0.1)
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        self.assertTrue(follower.update(Pose2D(0.95, 0.0, 0.0)).done)

    def test_high_curvature_scales_linear_speed_to_keep_angular_limit(self):
        follower = self.follower(
            lookahead_m=0.1,
            linear_speed_m_s=0.5,
            max_angular_speed_rad_s=0.2,
            rotate_in_place_threshold_rad=math.radians(50.0),
        )
        follower.set_path([(0.0, 0.0), (0.1, 0.1), (0.0, 0.2)])
        command = follower.update(Pose2D())
        self.assertLess(command.linear_m_s, 0.5)
        self.assertLessEqual(abs(command.angular_rad_s), 0.2)

    def test_wheel_speed_conversion_preserves_ratio_when_limited(self):
        wheel_command = differential_wheel_speeds(
            VelocityCommand(0.2, 1.0), track_width_m=0.2, max_wheel_speed_m_s=0.2
        )
        self.assertAlmostEqual(wheel_command.left_m_s, 1.0 / 15.0)
        self.assertAlmostEqual(wheel_command.right_m_s, 0.2)

    def test_wheel_speed_conversion_rejects_nonfinite_command(self):
        with self.assertRaises(ValueError):
            differential_wheel_speeds(
                VelocityCommand(float("nan"), 0.0), track_width_m=0.2,
                max_wheel_speed_m_s=0.2,
            )

    def test_invalid_path_and_config_are_rejected(self):
        with self.assertRaises(ValueError):
            FollowerConfig(0.0, 0.2, 1.0)
        follower = self.follower()
        with self.assertRaises(ValueError):
            follower.set_path([(0.0, 0.0), (float("nan"), 1.0)])

    def test_follower_rejects_invalid_pose(self):
        follower = self.follower()
        follower.set_path([(0.0, 0.0), (1.0, 0.0)])
        with self.assertRaises(ValueError):
            follower.update(Pose2D(yaw=math.inf))


if __name__ == "__main__":
    unittest.main()
