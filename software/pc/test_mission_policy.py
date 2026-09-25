import unittest

from mission_policy import MissionPolicy, MissionProgress, Target, TargetKind


def target(target_id, kind, x=0.0, y=0.0, confidence=0.9):
    return Target(target_id, kind, x, y, confidence)


class MissionPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = MissionPolicy()

    def test_first_delivery_only_accepts_one_confident_ordinary(self):
        progress = MissionProgress()
        self.assertEqual(
            self.policy.evaluate(target("ordinary", TargetKind.ORDINARY), progress).reason,
            "first_ordinary",
        )
        for kind in (TargetKind.CORE, TargetKind.CASUALTY):
            self.assertFalse(self.policy.evaluate(target(kind.value, kind), progress).eligible)
        self.assertFalse(
            self.policy.evaluate(target("uncertain", TargetKind.ORDINARY, confidence=0.59), progress).eligible
        )

    def test_first_delivery_does_not_select_second_object_while_carrying(self):
        progress = MissionProgress(carried_material_count=1)
        result = self.policy.evaluate(target("second", TargetKind.ORDINARY), progress)
        self.assertEqual(result.reason, "first_delivery_allows_exactly_one")
        self.assertFalse(result.eligible)

    def test_danger_and_opponent_are_obstacles_never_collection_targets(self):
        progress = MissionProgress(first_ordinary_delivered=True, delivered_material_count=1)
        for kind in (TargetKind.DANGER, TargetKind.OPPONENT_ROBOT):
            result = self.policy.evaluate(target(kind.value, kind), progress)
            self.assertFalse(result.eligible)
            self.assertEqual(result.reason, "obstacle_only")

    def test_material_count_includes_current_cargo_and_stops_at_three(self):
        policy = MissionPolicy(max_materials=3)
        allowed = MissionProgress(first_ordinary_delivered=True, delivered_material_count=1, carried_material_count=1)
        blocked = MissionProgress(first_ordinary_delivered=True, delivered_material_count=2, carried_material_count=1)
        self.assertTrue(policy.evaluate(target("core", TargetKind.CORE), allowed).eligible)
        self.assertEqual(
            policy.evaluate(target("ordinary", TargetKind.ORDINARY), blocked).reason,
            "material_limit_reached",
        )

    def test_casualty_trip_requires_empty_material_cargo(self):
        empty = MissionProgress(first_ordinary_delivered=True, delivered_material_count=1)
        loaded = MissionProgress(
            first_ordinary_delivered=True,
            delivered_material_count=1,
            carried_material_count=1,
        )
        self.assertTrue(self.policy.evaluate(target("casualty", TargetKind.CASUALTY), empty).eligible)
        self.assertEqual(
            self.policy.evaluate(target("casualty", TargetKind.CASUALTY), loaded).reason,
            "casualty_requires_empty_material_cargo",
        )

    def test_casualty_onboard_excludes_every_new_pickup(self):
        progress = MissionProgress(
            first_ordinary_delivered=True,
            delivered_material_count=1,
            casualty_onboard=True,
        )
        for kind in (TargetKind.ORDINARY, TargetKind.CORE, TargetKind.CASUALTY):
            self.assertEqual(
                self.policy.evaluate(target(kind.value, kind), progress).reason,
                "casualty_trip_is_exclusive",
            )

    def test_select_nearest_eligible_and_confidence_tie_break(self):
        progress = MissionProgress(first_ordinary_delivered=True, delivered_material_count=1)
        candidates = [
            target("danger", TargetKind.DANGER, 0.1, 0.0),
            target("far", TargetKind.ORDINARY, 2.0, 0.0),
            target("tie-low", TargetKind.CORE, 1.0, 0.0, 0.7),
            target("tie-high", TargetKind.ORDINARY, 0.0, 1.0, 0.95),
        ]
        selected = self.policy.select_nearest(candidates, (0.0, 0.0), progress)
        self.assertEqual(selected.target_id, "tie-high")

    def test_select_returns_none_when_nothing_is_legal(self):
        candidates = [target("danger", TargetKind.DANGER), target("core", TargetKind.CORE)]
        self.assertIsNone(self.policy.select_nearest(candidates, (0.0, 0.0), MissionProgress()))

    def test_progress_rejects_conflicting_cargo_state(self):
        with self.assertRaises(ValueError):
            MissionProgress(first_ordinary_delivered=True, casualty_onboard=True, carried_material_count=1)

    def test_target_rejects_invalid_confidence_and_coordinates(self):
        with self.assertRaises(ValueError):
            target("bad-conf", TargetKind.ORDINARY, confidence=1.1)
        with self.assertRaises(ValueError):
            target("bad-pos", TargetKind.ORDINARY, x=float("nan"))


if __name__ == "__main__":
    unittest.main()
