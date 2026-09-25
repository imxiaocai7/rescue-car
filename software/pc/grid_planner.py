"""Configurable 2-D occupancy grid and conservative A* path planning."""

from heapq import heappop, heappush
import math


Cell = tuple[int, int]  # (column, row)


class GridMap:
    """Square-cell field map. World coordinates are metres; cells are (x, y)."""

    def __init__(
        self,
        width_m: float = 3.0,
        height_m: float = 3.0,
        resolution_m: float = 0.05,
        *,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
    ) -> None:
        values = (width_m, height_m, resolution_m, origin_x, origin_y)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("grid dimensions and origin must be finite")
        if width_m <= 0 or height_m <= 0 or resolution_m <= 0:
            raise ValueError("grid dimensions and resolution must be positive")

        self.width_m = width_m
        self.height_m = height_m
        self.resolution_m = resolution_m
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.columns = self._cell_count(width_m, resolution_m)
        self.rows = self._cell_count(height_m, resolution_m)
        self._blocked = [bytearray(self.columns) for _ in range(self.rows)]

    @staticmethod
    def _cell_count(length_m: float, resolution_m: float) -> int:
        count = round(length_m / resolution_m)
        if count < 1 or not math.isclose(
            count * resolution_m, length_m, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("width and height must be integer multiples of resolution")
        return count

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.columns and 0 <= y < self.rows

    def _require_cell(self, cell: Cell) -> None:
        if not self.in_bounds(cell):
            raise IndexError(f"cell outside grid: {cell}")

    def world_to_cell(self, x_m: float, y_m: float) -> Cell:
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise ValueError("world coordinates must be finite")
        x_index = math.floor((x_m - self.origin_x) / self.resolution_m)
        y_index = math.floor((y_m - self.origin_y) / self.resolution_m)
        cell = (x_index, y_index)
        self._require_cell(cell)
        return cell

    def cell_to_world(self, cell: Cell) -> tuple[float, float]:
        self._require_cell(cell)
        x, y = cell
        return (
            self.origin_x + (x + 0.5) * self.resolution_m,
            self.origin_y + (y + 0.5) * self.resolution_m,
        )

    def set_blocked(self, cell: Cell, blocked: bool = True) -> None:
        self._require_cell(cell)
        x, y = cell
        self._blocked[y][x] = 1 if blocked else 0

    def is_blocked(self, cell: Cell) -> bool:
        if not self.in_bounds(cell):
            return True
        x, y = cell
        return bool(self._blocked[y][x])

    def add_circle_obstacle(
        self,
        center_x_m: float,
        center_y_m: float,
        radius_m: float,
        *,
        inflation_m: float = 0.0,
    ) -> None:
        """Mark cells touched by a circle expanded by robot radius/margin.

        The cell half-diagonal is included so occupied squares touching the
        expanded obstacle are conservatively blocked.
        """
        self._validate_obstacle(center_x_m, center_y_m, radius_m, inflation_m)
        extent = radius_m + inflation_m + self.resolution_m * math.sqrt(2.0) / 2.0
        x0 = max(0, math.floor((center_x_m - extent - self.origin_x) / self.resolution_m))
        x1 = min(self.columns - 1, math.floor((center_x_m + extent - self.origin_x) / self.resolution_m))
        y0 = max(0, math.floor((center_y_m - extent - self.origin_y) / self.resolution_m))
        y1 = min(self.rows - 1, math.floor((center_y_m + extent - self.origin_y) / self.resolution_m))
        limit_sq = extent * extent
        for row in range(y0, y1 + 1):
            for column in range(x0, x1 + 1):
                cell_x, cell_y = self.cell_to_world((column, row))
                if (cell_x - center_x_m) ** 2 + (cell_y - center_y_m) ** 2 <= limit_sq:
                    self._blocked[row][column] = 1

    def add_rect_obstacle(
        self,
        x_min_m: float,
        y_min_m: float,
        x_max_m: float,
        y_max_m: float,
        *,
        inflation_m: float = 0.0,
    ) -> None:
        """Mark cells touched by an axis-aligned rectangle plus margin."""
        values = (x_min_m, y_min_m, x_max_m, y_max_m, inflation_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("obstacle coordinates must be finite")
        if x_min_m >= x_max_m or y_min_m >= y_max_m or inflation_m < 0:
            raise ValueError("rectangle extents must be ordered and inflation nonnegative")

        half_cell = self.resolution_m / 2.0
        x_min = x_min_m - inflation_m - half_cell
        x_max = x_max_m + inflation_m + half_cell
        y_min = y_min_m - inflation_m - half_cell
        y_max = y_max_m + inflation_m + half_cell
        x0 = max(0, math.floor((x_min - self.origin_x) / self.resolution_m))
        x1 = min(self.columns - 1, math.floor((x_max - self.origin_x) / self.resolution_m))
        y0 = max(0, math.floor((y_min - self.origin_y) / self.resolution_m))
        y1 = min(self.rows - 1, math.floor((y_max - self.origin_y) / self.resolution_m))
        for row in range(y0, y1 + 1):
            for column in range(x0, x1 + 1):
                cell_x, cell_y = self.cell_to_world((column, row))
                overlaps_x = (
                    x_min_m - inflation_m <= cell_x + half_cell
                    and cell_x - half_cell <= x_max_m + inflation_m
                )
                overlaps_y = (
                    y_min_m - inflation_m <= cell_y + half_cell
                    and cell_y - half_cell <= y_max_m + inflation_m
                )
                if overlaps_x and overlaps_y:
                    self._blocked[row][column] = 1

    def block_boundary_margin(self, margin_m: float) -> None:
        """Keep cell centers far enough inside the field for the robot footprint."""
        if not math.isfinite(margin_m) or margin_m < 0:
            raise ValueError("boundary margin must be finite and nonnegative")
        clearance = margin_m + self.resolution_m * math.sqrt(2.0) / 2.0
        right = self.origin_x + self.width_m
        top = self.origin_y + self.height_m
        for row in range(self.rows):
            for column in range(self.columns):
                x, y = self.cell_to_world((column, row))
                if (
                    x - self.origin_x < clearance
                    or right - x < clearance
                    or y - self.origin_y < clearance
                    or top - y < clearance
                ):
                    self._blocked[row][column] = 1

    @staticmethod
    def _validate_obstacle(x: float, y: float, radius: float, inflation: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, radius, inflation)):
            raise ValueError("obstacle geometry must be finite")
        if radius < 0 or inflation < 0:
            raise ValueError("radius and inflation must be nonnegative")

    def find_path(self, start: Cell, goal: Cell, *, diagonal: bool = True) -> list[Cell] | None:
        """Return an optimal A* path including endpoints, or None if unreachable."""
        self._require_cell(start)
        self._require_cell(goal)
        if self.is_blocked(start) or self.is_blocked(goal):
            return None
        if start == goal:
            return [start]

        if diagonal:
            moves = (
                (1, 0, 1.0),
                (-1, 0, 1.0),
                (0, 1, 1.0),
                (0, -1, 1.0),
                (1, 1, math.sqrt(2.0)),
                (1, -1, math.sqrt(2.0)),
                (-1, 1, math.sqrt(2.0)),
                (-1, -1, math.sqrt(2.0)),
            )
        else:
            moves = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0))

        frontier: list[tuple[float, float, int, int]] = []
        heappush(frontier, (self._heuristic(start, goal, diagonal), 0.0, start[0], start[1]))
        came_from: dict[Cell, Cell] = {}
        cost_so_far: dict[Cell, float] = {start: 0.0}

        while frontier:
            _, queued_cost, x, y = heappop(frontier)
            current = (x, y)
            if queued_cost > cost_so_far.get(current, math.inf) + 1e-12:
                continue
            if current == goal:
                return self._reconstruct(came_from, start, goal)

            for dx, dy, step_cost in moves:
                neighbor = (x + dx, y + dy)
                if self.is_blocked(neighbor):
                    continue
                if dx and dy:
                    # Do not squeeze diagonally between blocked squares.
                    if self.is_blocked((x + dx, y)) or self.is_blocked((x, y + dy)):
                        continue

                new_cost = cost_so_far[current] + step_cost
                if new_cost + 1e-12 >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + self._heuristic(neighbor, goal, diagonal)
                heappush(frontier, (priority, new_cost, neighbor[0], neighbor[1]))
        return None

    def find_world_path(
        self,
        start_xy: tuple[float, float],
        goal_xy: tuple[float, float],
        *,
        diagonal: bool = True,
    ) -> list[tuple[float, float]] | None:
        """Plan between world points and return waypoint cell centers in metres."""
        cells = self.find_path(
            self.world_to_cell(*start_xy),
            self.world_to_cell(*goal_xy),
            diagonal=diagonal,
        )
        if cells is None:
            return None
        return [self.cell_to_world(cell) for cell in cells]

    @staticmethod
    def _heuristic(start: Cell, goal: Cell, diagonal: bool) -> float:
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        if not diagonal:
            return float(dx + dy)
        diagonal_steps = min(dx, dy)
        straight_steps = max(dx, dy) - diagonal_steps
        return diagonal_steps * math.sqrt(2.0) + straight_steps

    @staticmethod
    def _reconstruct(came_from: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path.reverse()
        return path

    @staticmethod
    def path_cost(path: list[Cell]) -> float:
        """Return the unit-grid path cost for a path returned by find_path."""
        cost = 0.0
        for current, following in zip(path, path[1:]):
            dx = abs(current[0] - following[0])
            dy = abs(current[1] - following[1])
            if max(dx, dy) != 1 or dx + dy == 0:
                raise ValueError("path contains non-adjacent cells")
            cost += math.sqrt(2.0) if dx and dy else 1.0
        return cost
