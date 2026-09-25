"""Combine rule-gated target selection with obstacle-aware A* routes."""

from dataclasses import dataclass
from typing import Iterable

from grid_planner import GridMap
from mission_policy import MissionPolicy, MissionProgress, Target


@dataclass(frozen=True)
class TargetRoute:
    target: Target
    waypoints_m: tuple[tuple[float, float], ...]
    route_length_m: float


class AutonomousPlanner:
    """Rank legal, reachable targets by actual grid path length.

    The occupancy map must encode static and non-collectible obstacles. Keep a
    candidate's own cell unblocked when it is the requested destination; this
    module does not guess the robot's pickup standoff or approach heading.
    """

    def __init__(
        self,
        grid: GridMap,
        mission_policy: MissionPolicy | None = None,
        *,
        diagonal: bool = True,
    ) -> None:
        self.grid = grid
        self.mission_policy = mission_policy or MissionPolicy()
        self.diagonal = diagonal

    def rank_targets(
        self,
        targets: Iterable[Target],
        robot_xy: tuple[float, float],
        progress: MissionProgress,
    ) -> list[TargetRoute]:
        """Return legal/reachable targets, shortest route first.

        Equal-length routes prefer higher detector confidence, then stable
        target ID order. Candidates outside the map, inside blocked cells,
        illegal under mission rules, or separated by obstacles are excluded.
        """
        start = self.grid.world_to_cell(*robot_xy)
        routes: list[TargetRoute] = []
        for target in targets:
            if not self.mission_policy.evaluate(target, progress).eligible:
                continue
            try:
                goal = self.grid.world_to_cell(target.x, target.y)
            except IndexError:
                continue
            path = self.grid.find_path(start, goal, diagonal=self.diagonal)
            if path is None:
                continue
            routes.append(
                TargetRoute(
                    target=target,
                    waypoints_m=tuple(self.grid.cell_to_world(cell) for cell in path),
                    route_length_m=self.grid.path_cost(path) * self.grid.resolution_m,
                )
            )
        routes.sort(key=lambda route: (route.route_length_m, -route.target.confidence, route.target.target_id))
        return routes

    def plan_next_target(
        self,
        targets: Iterable[Target],
        robot_xy: tuple[float, float],
        progress: MissionProgress,
    ) -> TargetRoute | None:
        routes = self.rank_targets(targets, robot_xy, progress)
        return routes[0] if routes else None
