import math
import unittest

from grid_planner import GridMap


class GridPlannerTests(unittest.TestCase):
    def test_world_cell_round_trip_and_bounds(self):
        grid = GridMap(1.0, 0.5, 0.1, origin_x=-0.5, origin_y=1.0)
        cell = grid.world_to_cell(-0.25, 1.35)
        self.assertEqual(cell, (2, 3))
        self.assertEqual(grid.cell_to_world(cell), (-0.25, 1.35))
        with self.assertRaises(IndexError):
            grid.world_to_cell(0.5, 1.1)

    def test_world_path_returns_cell_center_waypoints(self):
        grid = GridMap(1.0, 1.0, 0.1)
        path = grid.find_world_path((0.01, 0.01), (0.81, 0.51))
        self.assertIsNotNone(path)
        self.assertAlmostEqual(path[0][0], 0.05)
        self.assertAlmostEqual(path[0][1], 0.05)
        self.assertAlmostEqual(path[-1][0], 0.85)
        self.assertAlmostEqual(path[-1][1], 0.55)

    def test_diagonal_astar_uses_octile_shortest_path(self):
        grid = GridMap(1.0, 1.0, 0.1)
        path = grid.find_path((0, 0), (8, 5))
        self.assertIsNotNone(path)
        self.assertAlmostEqual(grid.path_cost(path), 5 * math.sqrt(2) + 3)

    def test_path_routes_around_inflated_wall_gap(self):
        grid = GridMap(0.8, 0.6, 0.1)
        for row in range(5):
            grid.set_blocked((3, row))
        path = grid.find_path((1, 2), (6, 2))
        self.assertIsNotNone(path)
        self.assertTrue(all(not grid.is_blocked(cell) for cell in path))
        self.assertIn((3, 5), path)

    def test_diagonal_move_cannot_cut_a_blocked_corner(self):
        grid = GridMap(0.3, 0.3, 0.1)
        grid.set_blocked((1, 0))
        grid.set_blocked((0, 1))
        self.assertIsNone(grid.find_path((0, 0), (1, 1)))

    def test_barrier_without_gap_is_unreachable(self):
        grid = GridMap(0.5, 0.5, 0.1)
        for row in range(grid.rows):
            grid.set_blocked((2, row))
        self.assertIsNone(grid.find_path((0, 2), (4, 2)))

    def test_obstacle_inflation_blocks_robot_clearance(self):
        grid = GridMap(1.0, 1.0, 0.1)
        grid.add_circle_obstacle(0.45, 0.45, 0.0, inflation_m=0.11)
        self.assertTrue(grid.is_blocked(grid.world_to_cell(0.45, 0.45)))
        self.assertTrue(grid.is_blocked(grid.world_to_cell(0.55, 0.45)))
        self.assertTrue(grid.is_blocked(grid.world_to_cell(0.55, 0.55)))
        self.assertFalse(grid.is_blocked(grid.world_to_cell(0.75, 0.45)))

    def test_rectangle_obstacle_applies_margin(self):
        grid = GridMap(1.0, 1.0, 0.1)
        grid.add_rect_obstacle(0.4, 0.4, 0.5, 0.5, inflation_m=0.1)
        self.assertTrue(grid.is_blocked(grid.world_to_cell(0.35, 0.45)))
        self.assertTrue(grid.is_blocked(grid.world_to_cell(0.55, 0.45)))
        self.assertFalse(grid.is_blocked(grid.world_to_cell(0.75, 0.45)))

    def test_boundary_margin_keeps_robot_footprint_inside_field(self):
        grid = GridMap(0.6, 0.6, 0.1)
        grid.block_boundary_margin(0.1)
        self.assertTrue(grid.is_blocked((0, 3)))
        self.assertTrue(grid.is_blocked((5, 3)))
        self.assertFalse(grid.is_blocked((3, 3)))
        path = grid.find_path((2, 3), (3, 3))
        self.assertIsNotNone(path)
        self.assertTrue(all(2 <= x <= 3 and 2 <= y <= 4 for x, y in path))

    def test_same_cell_path_and_blocked_endpoints(self):
        grid = GridMap(0.4, 0.4, 0.1)
        self.assertEqual(grid.find_path((1, 1), (1, 1)), [(1, 1)])
        grid.set_blocked((2, 2))
        self.assertIsNone(grid.find_path((1, 1), (2, 2)))

    def test_grid_dimensions_must_tile_exactly(self):
        with self.assertRaises(ValueError):
            GridMap(1.0, 1.0, 0.3)


if __name__ == "__main__":
    unittest.main()
