"""Small data contracts shared by the navigation core."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from collections.abc import Iterable, Sequence


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float
    yaw: float | None = None

    def to_dict(self) -> dict[str, float]:
        result = {"x": self.x, "y": self.y}
        if self.yaw is not None:
            result["yaw"] = self.yaw
        return result


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

    def to_point(self) -> Point2D:
        return Point2D(self.x, self.y)


@dataclass(frozen=True)
class Velocity:
    linear: float
    angular: float


STOP = Velocity(0.0, 0.0)


class State(str, Enum):
    NAVIGATING = "NAVIGATING"
    SUCCEEDED = "SUCCEEDED"
    CANCELLED = "CANCELLED"
    BLOCKED = "BLOCKED"
    ERROR = "ERROR"


class GoalPhase(str, Enum):
    TRACKING = "TRACKING"
    FINAL = "FINAL"


@dataclass(frozen=True)
class GlobalPlan:
    path: tuple[Point2D, ...]


@dataclass(frozen=True)
class ControlDecision:
    command: Velocity
    clearance_m: float | None
    safe: bool
    escaping: bool


@dataclass(frozen=True)
class NavigationInput:
    pose: Pose2D
    velocity: Velocity
    scan_sequence: int
    obstacles_robot: tuple[Point2D, ...]


@dataclass(frozen=True)
class NavigationResult:
    state: State
    command: Velocity
    pose: Pose2D
    goal: Point2D
    path: tuple[Point2D, ...]
    clearance_m: float | None
    error: str | None = None


class SensorDataError(RuntimeError):
    pass


def distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lerp(start: Point2D, end: Point2D, amount: float) -> Point2D:
    return Point2D(
        start.x + (end.x - start.x) * amount,
        start.y + (end.y - start.y) * amount,
    )


def transform_point(pose: Pose2D, point: Point2D) -> Point2D:
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return Point2D(
        pose.x + point.x * cosine - point.y * sine,
        pose.y + point.x * sine + point.y * cosine,
    )


def transform_points(pose: Pose2D, points: Iterable[Point2D]) -> tuple[Point2D, ...]:
    return tuple(transform_point(pose, point) for point in points)


def resample_path(path: Sequence[Point2D], spacing: float) -> tuple[Point2D, ...]:
    if spacing <= 0.0:
        raise ValueError("path spacing must be positive")
    if len(path) < 2:
        return tuple(path)
    result = [path[0]]
    for start, end in zip(path, path[1:]):
        divisions = max(1, math.ceil(distance(start, end) / spacing))
        result.extend(lerp(start, end, index / divisions) for index in range(1, divisions + 1))
    result[-1] = path[-1]
    return tuple(result)


def target_point(point: Point2D, path: Sequence[Point2D], lookahead: float) -> Point2D:
    if not path:
        raise ValueError("path is empty")
    index = min(range(len(path)), key=lambda item: distance(point, path[item]))
    travelled = distance(point, path[index])
    for current in range(index, len(path) - 1):
        travelled += distance(path[current], path[current + 1])
        if travelled >= lookahead:
            return path[current + 1]
    return path[-1]


def min_distance(point: Point2D, others: Iterable[Point2D]) -> float:
    return min((distance(point, other) for other in others), default=math.inf)
