import math
import unittest

from odometry import Pose2D
from pixel_ground import GroundPlaneProjector, robot_to_field


class PixelGroundTests(unittest.TestCase):
    def projector(self):
        return GroundPlaneProjector.from_correspondences(
            [(0, 0), (100, 0), (100, 100), (0, 100)],
            [(0, 0), (1, 0), (1, 1), (0, 1)],
            image_size=(100, 100),
        )

    def test_four_point_calibration_interpolates_planar_coordinates(self):
        projector = self.projector()
        x, y = projector.pixel_to_robot((25, 70))
        self.assertAlmostEqual(x, 0.25)
        self.assertAlmostEqual(y, 0.7)

    def test_perspective_calibration_recovers_known_mapping(self):
        # Ground x = u/(100 + u), y = v/(100 + u).
        projector = GroundPlaneProjector.from_correspondences(
            [(0, 0), (100, 0), (100, 100), (0, 100)],
            [(0, 0), (0.5, 0), (0.5, 0.5), (0, 1)],
        )
        x, y = projector.pixel_to_robot((50, 50))
        self.assertAlmostEqual(x, 1.0 / 3.0)
        self.assertAlmostEqual(y, 1.0 / 3.0)

    def test_box_uses_bottom_center_as_ground_contact(self):
        point = self.projector().box_footpoint_to_robot((20, 10, 60, 80))
        self.assertAlmostEqual(point[0], 0.4)
        self.assertAlmostEqual(point[1], 0.8)

    def test_robot_relative_point_transforms_to_field_frame(self):
        point = robot_to_field((1.0, 0.0), Pose2D(2.0, 3.0, math.pi / 2))
        self.assertAlmostEqual(point[0], 2.0)
        self.assertAlmostEqual(point[1], 4.0)

    def test_calibration_rejects_collinear_and_wrong_count_points(self):
        with self.assertRaises(ValueError):
            GroundPlaneProjector.from_correspondences(
                [(0, 0), (1, 0), (2, 0), (3, 0)],
                [(0, 0), (1, 0), (1, 1), (0, 1)],
            )
        with self.assertRaises(ValueError):
            GroundPlaneProjector.from_correspondences([(0, 0)], [(0, 0)])

    def test_invalid_pixel_box_and_pose_are_rejected(self):
        projector = self.projector()
        with self.assertRaises(ValueError):
            projector.pixel_to_robot((101, 20))
        with self.assertRaises(ValueError):
            projector.box_footpoint_to_robot((20, 30, 10, 40))
        with self.assertRaises(ValueError):
            robot_to_field((0.0, 0.0), Pose2D(yaw=float("nan")))

    def test_horizon_pixels_are_rejected(self):
        projector = GroundPlaneProjector(
            ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, -5.0))
        )
        with self.assertRaises(ValueError):
            projector.pixel_to_robot((5.0, 2.0))


if __name__ == "__main__":
    unittest.main()
