"""Calibrate pixel-to-ground projection and robot-local to field coordinates."""

from collections.abc import Sequence
from dataclasses import dataclass
import math

from odometry import Pose2D


PixelPoint = tuple[float, float]
GroundPoint = tuple[float, float]
Matrix3 = tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class GroundPlaneProjector:
    """Planar homography; ground coordinates use robot x-forward/y-left axes."""

    matrix: Matrix3
    image_size: tuple[int, int] | None = None

    def __post_init__(self) -> None:
        if len(self.matrix) != 3 or any(len(row) != 3 for row in self.matrix):
            raise ValueError("homography matrix must be 3 by 3")
        normalized = tuple(tuple(float(value) for value in row) for row in self.matrix)
        if not all(math.isfinite(value) for row in normalized for value in row):
            raise ValueError("homography values must be finite")
        object.__setattr__(self, "matrix", normalized)
        if self.image_size is not None:
            width, height = self.image_size
            if (
                isinstance(width, bool)
                or isinstance(height, bool)
                or not isinstance(width, int)
                or not isinstance(height, int)
                or width <= 0
                or height <= 0
            ):
                raise ValueError("image size must contain positive integer width and height")

    @classmethod
    def from_correspondences(
        cls,
        image_points: Sequence[PixelPoint],
        robot_ground_points: Sequence[GroundPoint],
        *,
        image_size: tuple[int, int] | None = None,
    ) -> "GroundPlaneProjector":
        """Solve an exact four-point planar calibration in the robot frame.

        Use four distinct, non-collinear points on the floor. Their paired
        robot-frame coordinates must be measured in metres with x forward and
        y left. More samples can be captured for validation, but this initial
        solver intentionally uses exactly four well-distributed anchors.
        """
        if len(image_points) != 4 or len(robot_ground_points) != 4:
            raise ValueError("exactly four image/ground point pairs are required")
        image = tuple(_finite_point(point, "image point") for point in image_points)
        ground = tuple(_finite_point(point, "ground point") for point in robot_ground_points)

        equations: list[list[float]] = []
        for (u, v), (x, y) in zip(image, ground):
            equations.append([u, v, 1.0, 0.0, 0.0, 0.0, -x * u, -x * v, x])
            equations.append([0.0, 0.0, 0.0, u, v, 1.0, -y * u, -y * v, y])

        solution = _solve_linear_system(equations)
        matrix: Matrix3 = (
            (solution[0], solution[1], solution[2]),
            (solution[3], solution[4], solution[5]),
            (solution[6], solution[7], 1.0),
        )
        return cls(matrix, image_size)

    def pixel_to_robot(self, point: PixelPoint) -> GroundPoint:
        """Project one ground-contact image point into the robot-local plane."""
        u, v = _finite_point(point, "pixel point")
        if self.image_size is not None:
            width, height = self.image_size
            if not 0.0 <= u <= width or not 0.0 <= v <= height:
                raise ValueError("pixel point is outside the calibrated image")
        h = self.matrix
        denominator = h[2][0] * u + h[2][1] * v + h[2][2]
        if not math.isfinite(denominator) or abs(denominator) <= 1e-12:
            raise ValueError("pixel maps to the homography horizon")
        x = (h[0][0] * u + h[0][1] * v + h[0][2]) / denominator
        y = (h[1][0] * u + h[1][1] * v + h[1][2]) / denominator
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("projected ground coordinate is not finite")
        return x, y

    def box_footpoint_to_robot(self, xyxy: Sequence[float]) -> GroundPoint:
        """Map a detector box's bottom-center contact point into robot axes."""
        if len(xyxy) != 4:
            raise ValueError("bounding box must contain x1, y1, x2, y2")
        x1, y1, x2, y2 = (float(value) for value in xyxy)
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            raise ValueError("bounding box coordinates must be finite")
        if x1 > x2 or y1 > y2:
            raise ValueError("bounding box corners must be ordered")
        return self.pixel_to_robot(((x1 + x2) / 2.0, y2))


def robot_to_field(point: GroundPoint, pose: Pose2D) -> GroundPoint:
    """Transform robot-local x-forward/y-left metres into the map frame."""
    x_local, y_local = _finite_point(point, "robot point")
    if not all(math.isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
        raise ValueError("robot pose must be finite")
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return (
        pose.x + cosine * x_local - sine * y_local,
        pose.y + sine * x_local + cosine * y_local,
    )


def _finite_point(point: Sequence[float], name: str) -> tuple[float, float]:
    if len(point) != 2:
        raise ValueError(f"{name} must contain two coordinates")
    x, y = float(point[0]), float(point[1])
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"{name} coordinates must be finite")
    return x, y


def _solve_linear_system(rows: list[list[float]]) -> list[float]:
    """Solve an 8x8 system by scaled partial-pivot Gauss-Jordan elimination."""
    size = len(rows)
    if size != 8 or any(len(row) != size + 1 for row in rows):
        raise ValueError("homography calibration requires an 8x8 system")
    augmented = []
    for row in rows:
        scale = max(abs(value) for value in row[:-1])
        if scale <= 1e-15:
            raise ValueError("degenerate point correspondences")
        augmented.append([value / scale for value in row])

    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        pivot_value = augmented[pivot][column]
        if abs(pivot_value) <= 1e-12:
            raise ValueError("degenerate or nearly singular calibration points")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        pivot_value = augmented[column][column]
        augmented[column] = [value / pivot_value for value in augmented[column]]
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor:
                augmented[row] = [
                    value - factor * pivot_value_item
                    for value, pivot_value_item in zip(augmented[row], augmented[column])
                ]
    solution = [augmented[index][-1] for index in range(size)]
    if not all(math.isfinite(value) for value in solution):
        raise ValueError("calibration produced non-finite homography values")
    return solution
