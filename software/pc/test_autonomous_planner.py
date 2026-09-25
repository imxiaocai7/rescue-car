import unittest

from autonomous_planner import AutonomousPlanner
from grid_planner import GridMap
from mission_policy import MissionProgress, Target, TargetKind


def target(target_id, kind, x, y, confidence=0.9):
    return Target(target_id, kind, x, y, confidence)


class AutonomousPlannerTests(unittest.TestCase):
    def test_path_length_beats_euclidean_distance_around_a_wall(self):
        grid = GridMap(0.8, 0.8, 0.1)
        for row in range(7):
            grid.set_blocked((3, row))
        planner = AutonomousPlanner(grid)
        targets = [
            target("across-wall", TargetKind.ORDINARY, 0.45, 0.15),
            target("same-side", TargetKind.CORE, 0.25, 0.65),
        ]
        progress = MissionProgress(first_ordinary_delivered=True, delivered_material_count=1)
        routes = planner.rank_targets(targets, (0.15, 0.15), progress)
        self.assertEqual(routes[0].target.target_id, "same-side")
        self.assertGreater(routes[1].route_length_m, routes[0].route_length_m)
        self.assertTrue(all(len(route.waypoints_m) >= 2 for route in routes))

    def test_first_mission_rules_filter_targets_before_routing(self):
        planner = AutonomousPlanner(GridMap(1.0, 1.0, 0.1))
        targets = [
            target("core", TargetKind.CORE, 0.15, 0.15),
            target("casualty", TargetKind.CASUALTY, 0.25, 0.15),
            target("ordinary", TargetKind.ORDINARY, 0.85, 0.85),
            target("danger", TargetKind.DANGER, 0.12, 0.12),
        ]
        selected = planner.plan_next_target(targets, (0.1, 0.1), MissionProgress())
        self.assertEqual(selected.target.target_id, "ordinary")

    def test_blocked_and_unreachable_candidate_goals_are_skipped(self):
        grid = GridMap(0.5, 0.5, 0.1)
        for row in range(grid.rows):
            grid.set_blocked((2, row))
        grid.set_blocked((1, 1))
        planner = AutonomousPlanner(grid)
        progress = MissionProgress()
        candidates = [
            target("unreachable", TargetKind.ORDINARY, 0.35, 0.15),
            target("occupied", TargetKind.ORDINARY, 0.15, 0.15),
        ]
        self.assertEqual(planner.rank_targets(candidates, (0.05, 0.15), progress), [])

    def test_no_legal_candidates_returns_none(self):
        planner = AutonomousPlanner(GridMap(1.0, 1.0, 0.1))
        selected = planner.plan_next_target(
            [target("danger", TargetKind.DANGER, 0.5, 0.5)],
            (0.1, 0.1),
            MissionProgress(),
        )
        self.assertIsNone(selected)


if __name__ == "__main__":
    unittest.main()
