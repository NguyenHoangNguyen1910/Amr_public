"""Shared navigation data contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Protocol, TypedDict


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

    def to_dict(self) -> dict[str, float]:
        return {"linear": self.linear, "angular": self.angular}


STOP = Velocity(0.0, 0.0)


class State(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    READY = "READY"
    NAVIGATING = "NAVIGATING"
    RESETTING = "RESETTING"
    SUCCEEDED = "SUCCEEDED"
    CANCELLED = "CANCELLED"
    BLOCKED = "BLOCKED"
    ERROR = "ERROR"


class GoalPhase(str, Enum):
    """Closed-loop phases used to converge on a navigation goal."""

    TRACKING = "TRACKING"
    SETTLING_POSITION = "SETTLING_POSITION"
    SETTLING_YAW = "SETTLING_YAW"
    VERIFYING = "VERIFYING"


class DecisionReason(str, Enum):
    SELECTED = "selected"
    ESCAPING = "escaping"
    NO_SAFE_MAP_TRAJECTORY = "no_safe_map_trajectory"
    NO_SAFE_LIDAR_TRAJECTORY = "no_safe_lidar_trajectory"


@dataclass(frozen=True)
class GlobalPlan:
    """Global planner output in world coordinates."""

    raw: tuple[Point2D, ...]
    path: tuple[Point2D, ...]
    length_m: float


@dataclass(frozen=True)
class ControlDecision:
    """Local planner output with a candidate trajectory in world coordinates."""

    command: Velocity
    trajectory: tuple[Pose2D, ...]
    clearance_m: float | None
    safe: bool
    reason: DecisionReason


@dataclass(frozen=True)
class NavigationInput:
    """Validated input for one navigation cycle.

    Pose is in world coordinates; obstacle points are in robot coordinates.
    """

    pose: Pose2D
    velocity: Velocity
    scan_sequence: int
    obstacles_robot: tuple[Point2D, ...]


class ProgressField(Protocol):
    def progress_m(self, start: Point2D, end: Point2D) -> float: ...


class CostMapView(Protocol):
    """Map queries required by the local planner."""

    def is_free(self, point: Point2D) -> bool: ...

    def is_raw_free(self, point: Point2D) -> bool: ...

    def static_is_free(self, point: Point2D) -> bool: ...

    def clearance_at(self, point: Point2D) -> float: ...


class NavigationStatus(TypedDict):
    state: str
    message: str
    phase: str
    owns_control: bool
    start: dict[str, float] | None
    goal: dict[str, float] | None
    distance_to_goal_m: float | None
    yaw_error_to_goal_rad: float | None
    current_pose: dict[str, float] | None
    measured_velocity: dict[str, float] | None
    path_length_m: float | None
    waypoint_count: int
    global_path: list[dict[str, float]]
    command: dict[str, float]
    speed_mps: float
    minimum_clearance_m: float | None
    error: str | None
