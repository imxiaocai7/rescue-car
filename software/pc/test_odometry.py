import math
import unittest

from odometry import (
    DifferentialDriveConfig,
    DifferentialDriveOdometry,
    Pose2D,
    estimate_gyro_bias,
    modular_tick_delta,
    wrap_angle,
)


class OdometryTests(unittest.TestCase):
    def config(self, **overrides):
        values = {
            "wheel_radius_m": 0.05,
            "track_width_m": 0.20,
            "ticks_per_wheel_revolution": 100,
            "gyro_weight": 1.0,
        }
        values.update(overrides)
        return DifferentialDriveConfig(**values)

    def test_straight_motion_from_equal_ticks(self):
        odometry = DifferentialDriveOdometry(self.config())
        odometry.reset(0, 0)
        pose = odometry.update(100, 100, 0.0, 0.1)
        self.assertAlmostEqual(pose.x, 2.0 * math.pi * 0.05)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, 0.0)

    def test_gyro_integrates_turn_and_midpoint_updates_position(self):
        odometry = DifferentialDriveOdometry(self.config())
        odometry.reset(0, 0)
        pose = odometry.update(25, 25, math.pi / 2, 1.0)
        distance = 2.0 * math.pi * 0.05 * 0.25
        self.assertAlmostEqual(pose.yaw, math.pi / 2)
        self.assertAlmostEqual(pose.x, distance * math.cos(math.pi / 4))
        self.assertAlmostEqual(pose.y, distance * math.sin(math.pi / 4))

    def test_wheel_heading_corrects_gyro_drift(self):
        odometry = DifferentialDriveOdometry(self.config(gyro_weight=0.0))
        odometry.reset(0, 0)
        pose = odometry.update(-25, 25, 0.0, 1.0)
        expected = (2.0 * math.pi * 0.05 * 0.5) / 0.20
        self.assertAlmostEqual(pose.yaw, expected)
        self.assertAlmostEqual(pose.x, 0.0)

    def test_encoder_counter_rollover(self):
        self.assertEqual(modular_tick_delta(65530, 5, 65536), 11)
        self.assertEqual(modular_tick_delta(5, 65530, 65536), -11)
        odometry = DifferentialDriveOdometry(self.config(encoder_modulus=65536))
        odometry.reset(65530, 65530)
        pose = odometry.update(5, 5, 0.0, 0.1)
        self.assertAlmostEqual(pose.x, 11 * 2.0 * math.pi * 0.05 / 100)

    def test_reset_seeds_counters_without_artificial_motion(self):
        odometry = DifferentialDriveOdometry(self.config())
        initial = Pose2D(1.2, 0.8, 0.4)
        odometry.reset(456, 789, initial)
        pose = odometry.update(456, 789, 0.0, 0.1)
        self.assertAlmostEqual(pose.x, initial.x)
        self.assertAlmostEqual(pose.y, initial.y)
        self.assertAlmostEqual(pose.yaw, initial.yaw)

    def test_yaw_wraps_across_pi(self):
        odometry = DifferentialDriveOdometry(
            self.config(), pose=Pose2D(yaw=3.1)
        )
        odometry.reset(0, 0)
        pose = odometry.update(0, 0, 1.0, 0.2)
        self.assertAlmostEqual(pose.yaw, wrap_angle(3.3))
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)

    def test_bias_estimation_and_invalid_samples(self):
        self.assertAlmostEqual(estimate_gyro_bias([0.1, 0.2, 0.3]), 0.2)
        with self.assertRaises(ValueError):
            estimate_gyro_bias([])
        with self.assertRaises(ValueError):
            estimate_gyro_bias([0.1, float("nan")])

    def test_invalid_config_and_unseeded_update_fail_closed(self):
        with self.assertRaises(ValueError):
            self.config(track_width_m=0)
        with self.assertRaises(ValueError):
            self.config(ticks_per_wheel_revolution=0)
        with self.assertRaises(RuntimeError):
            DifferentialDriveOdometry(self.config()).update(0, 0, 0.0, 0.1)

    def test_bad_time_step_is_rejected(self):
        odometry = DifferentialDriveOdometry(self.config())
        odometry.reset(0, 0)
        with self.assertRaises(ValueError):
            odometry.update(1, 1, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
