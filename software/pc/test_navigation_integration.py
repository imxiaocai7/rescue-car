import math
import unittest

from grid_planner import GridMap
from odometry import Pose2D
from path_follower import FollowerConfig, PurePursuitFollower


class NavigationIntegrationTests(unittest.TestCase):
    def test_a_star_route_is_followed_without_entering_blocked_cells(self):
        grid = GridMap(2.0, 2.0, 0.05)
        grid.add_rect_obstacle(0.80, 0.20, 0.90, 0.80, inflation_m=0.20)
        start = grid.cell_to_world(grid.world_to_cell(0.50, 0.50))
        goal = (1.50, 0.50)
        path = grid.find_world_path(start, goal)
        self.assertIsNotNone(path)

        follower = PurePursuitFollower(
            FollowerConfig(
                lookahead_m=0.25,
                linear_speed_m_s=0.20,
                max_angular_speed_rad_s=1.2,
                goal_tolerance_m=0.05,
            )
        )
        follower.set_path(path)
        x, y = start
        yaw = 0.0  # Deliberately starts 90 degrees away from the first segment.
        dt = 0.02

        for _ in range(1_000):
            cell = grid.world_to_cell(x, y)
            self.assertFalse(grid.is_blocked(cell), f"entered blocked cell {cell}")
            command = follower.update(Pose2D(x, y, yaw))
            if command.done:
                break
            x += command.linear_m_s * math.cos(yaw) * dt
            y += command.linear_m_s * math.sin(yaw) * dt
            yaw += command.angular_rad_s * dt
        else:
            self.fail("follower did not finish the planned route")

        self.assertLessEqual(math.hypot(path[-1][0] - x, path[-1][1] - y), 0.05)


if __name__ == "__main__":
    unittest.main()
