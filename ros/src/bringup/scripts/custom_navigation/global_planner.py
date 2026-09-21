"""Static occupancy, runtime costmap, and goal-directed global planning."""

from __future__ import annotations

import heapq
import json
import math
from collections.abc import Iterator, Sequence
from enum import Enum
from pathlib import Path

import numpy as np

try:
    from scipy.ndimage import distance_transform_edt
except ImportError:  # pragma: no cover - only used in minimal non-ROS test envs.
    distance_transform_edt = None

from .config import NavigationConfig
from .helpers import distance, lerp, path_length, resample_path
from .types import GlobalPlan, Point2D

Cell = tuple[int, int]

_DIAGONAL = math.sqrt(2.0)
_NEIGHBOURS = (
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (1, 0, 1.0),
    (0, -1, 1.0),
    (-1, 1, _DIAGONAL),
    (1, 1, _DIAGONAL),
    (1, -1, _DIAGONAL),
    (-1, -1, _DIAGONAL),
)


# -- occupancy ----------------------------------------------------------------


def _clearance_metres(occupied: np.ndarray, resolution_m: float) -> np.ndarray:
    """Distance in metres from every cell centre to the nearest occupied cell."""

    if not occupied.any():
        return np.full(occupied.shape, math.inf)
    if distance_transform_edt is not None:
        # ``distance_transform_edt`` measures distance to the nearest zero.
        # Free cells are true and occupied cells are zero in this input.
        return distance_transform_edt(~occupied, sampling=resolution_m)
    rows, columns = np.nonzero(occupied)
    # Column offsets do not depend on the row, so square them once up front.
    column_squared = (np.arange(occupied.shape[1])[:, None] - columns) ** 2
    squared = np.empty(occupied.shape)
    for row in range(occupied.shape[0]):
        squared[row] = (column_squared + (rows - row) ** 2).min(axis=1)
    return np.sqrt(squared) * resolution_m


class OccupancyGrid:
    """A world-aligned grid and the queries planners run against it.

    ``raw`` is the obstacle geometry itself, ``clearance`` its distance
    transform, and ``occupied`` the inflated grid the robot centre must stay
    out of.
    """

    def __init__(
        self,
        raw: np.ndarray,
        *,
        min_x: float,
        min_y: float,
        resolution_m: float,
        inflation_radius_m: float,
    ):
        self.raw = raw
        self.min_x = min_x
        self.min_y = min_y
        self.resolution_m = resolution_m
        self.inflation_radius = inflation_radius_m
        self.height, self.width = raw.shape
        self.clearance = _clearance_metres(raw, resolution_m)
        # Sampling cell centres costs up to half a cell of accuracy; inflate by
        # that much more so the footprint never clips geometry between samples.
        self.occupied = self.clearance <= inflation_radius_m + 0.5 * resolution_m

    def world_to_grid(self, point: Point2D) -> Cell:
        return (
            math.floor((point.y - self.min_y) / self.resolution_m),
            math.floor((point.x - self.min_x) / self.resolution_m),
        )

    def grid_to_world(self, cell: Cell) -> Point2D:
        row, column = cell
        return Point2D(
            self.min_x + (column + 0.5) * self.resolution_m,
            self.min_y + (row + 0.5) * self.resolution_m,
        )

    def contains_cell(self, cell: Cell) -> bool:
        row, column = cell
        return 0 <= row < self.height and 0 <= column < self.width

    def is_free(self, point: Point2D) -> bool:
        """True where the robot centre may sit; outside the grid is never free."""

        cell = self.world_to_grid(point)
        return self.contains_cell(cell) and not bool(self.occupied[cell])

    def is_raw_free(self, point: Point2D) -> bool:
        """True outside the obstacle itself, ignoring the inflation band."""

        cell = self.world_to_grid(point)
        return self.contains_cell(cell) and not bool(self.raw[cell])

    def clearance_at(self, point: Point2D) -> float:
        cell = self.world_to_grid(point)
        return float(self.clearance[cell]) if self.contains_cell(cell) else 0.0

    def line_samples(self, start: Point2D, end: Point2D) -> Iterator[Point2D]:
        steps = max(1, math.ceil(distance(start, end) / (0.5 * self.resolution_m)))
        return (lerp(start, end, index / steps) for index in range(steps + 1))

    def line_is_free(self, start: Point2D, end: Point2D) -> bool:
        return all(self.is_free(point) for point in self.line_samples(start, end))


def load_static_map(path: Path, config: NavigationConfig) -> OccupancyGrid:
    """Rasterise the world's circular obstacles into an inflated grid.

    The map is a repository asset describing the SDF world, not a SLAM output.
    ``test_amr_navigation_map`` is what holds it in step with the SDF and with
    ``NavigationConfig``, so nothing is re-checked here at runtime.
    """

    payload = json.loads(path.read_text())
    bounds = payload["bounds"]
    resolution_m = float(payload["resolution_m"])
    min_x, min_y = float(bounds["min_x"]), float(bounds["min_y"])
    width = math.ceil((float(bounds["max_x"]) - min_x) / resolution_m)
    height = math.ceil((float(bounds["max_y"]) - min_y) / resolution_m)

    centre_x, centre_y = np.meshgrid(
        min_x + (np.arange(width) + 0.5) * resolution_m,
        min_y + (np.arange(height) + 0.5) * resolution_m,
    )
    raw = np.zeros((height, width), dtype=np.bool_)
    for obstacle in payload["obstacles"]:
        raw |= np.hypot(
            centre_x - float(obstacle["x"]), centre_y - float(obstacle["y"])
        ) <= float(obstacle["radius"])

    return OccupancyGrid(
        raw,
        min_x=min_x,
        min_y=min_y,
        resolution_m=resolution_m,
        inflation_radius_m=config.robot_radius_m + config.safety_margin_m,
    )


class CostMap:
    """Static occupancy plus LiDAR-seen obstacles that expire on a TTL.

    ``version`` increments whenever ``grid`` is replaced, which is how the
    global planner knows its cost field is stale.
    """

    def __init__(self, static: OccupancyGrid, obstacle_ttl_seconds: float):
        self.static = static
        self.grid = static
        self.obstacle_ttl_seconds = obstacle_ttl_seconds
        self._expires_at = np.zeros(static.raw.shape)
        self._dynamic = np.zeros(static.raw.shape, dtype=np.bool_)
        self.version = 0

    def update_world_obstacles(self, points: Sequence[Point2D], now: float) -> None:
        """Refresh TTLs for scan returns that known static geometry cannot explain."""

        for point in points:
            cell = self.static.world_to_grid(point)
            if self.static.contains_cell(cell) and not self.static.occupied[cell]:
                self._expires_at[cell] = now + self.obstacle_ttl_seconds
        self._set_dynamic(self._expires_at > now)

    def clear_dynamic_obstacles(self) -> None:
        self._expires_at.fill(0.0)
        self._set_dynamic(np.zeros_like(self._dynamic))

    def _set_dynamic(self, dynamic: np.ndarray) -> None:
        if np.array_equal(dynamic, self._dynamic):
            return
        self._dynamic = dynamic
        self.grid = (
            OccupancyGrid(
                self.static.raw | dynamic,
                min_x=self.static.min_x,
                min_y=self.static.min_y,
                resolution_m=self.static.resolution_m,
                inflation_radius_m=self.static.inflation_radius,
            )
            if dynamic.any()
            else self.static
        )
        self.version += 1

    # Planners query the combined grid; only the LiDAR filter in the local
    # planner asks the separate question of what the static map already knows.

    def is_free(self, point: Point2D) -> bool:
        return self.grid.is_free(point)

    def is_raw_free(self, point: Point2D) -> bool:
        return self.grid.is_raw_free(point)

    def clearance_at(self, point: Point2D) -> float:
        return self.grid.clearance_at(point)

    def static_is_free(self, point: Point2D) -> bool:
        return self.static.is_free(point)


# -- global planning ----------------------------------------------------------


class PlanningErrorCode(str, Enum):
    OUT_OF_BOUNDS = "OUT_OF_BOUNDS"
    START_BLOCKED = "START_BLOCKED"
    GOAL_BLOCKED = "GOAL_BLOCKED"
    NO_PATH = "NO_PATH"


class PlanningError(RuntimeError):
    def __init__(self, code: PlanningErrorCode, message: str):
        super().__init__(message)
        self.code = code


class GlobalPlanner:
    """Dijkstra cost-to-go field over the costmap for one world-frame goal.

    A whole field rather than a single route: the local planner scores its
    candidate arcs by how much cost-to-go each one removes, which needs the
    value at arbitrary poses, not just along one path.
    """

    def __init__(self, cost_map: CostMap, goal: Point2D, config: NavigationConfig):
        self.cost_map = cost_map
        self.goal = goal
        self.config = config
        self.goal_cell = cost_map.grid.world_to_grid(goal)
        if not cost_map.grid.contains_cell(self.goal_cell):
            raise PlanningError(PlanningErrorCode.OUT_OF_BOUNDS, "goal is outside the map")
        self.costs = np.empty((0, 0))
        self.next_cell: dict[Cell, Cell] = {}
        self._version = -1
        self.refresh()

    def refresh(self) -> None:
        """Rebuild the cost field when the costmap has changed underneath it."""

        if self.cost_map.grid.occupied[self.goal_cell]:
            raise PlanningError(PlanningErrorCode.GOAL_BLOCKED, "goal is in an inflated obstacle")
        if self._version != self.cost_map.version:
            self.costs, self.next_cell = self._dijkstra()
            self._version = self.cost_map.version

    def _neighbours(self, cell: Cell) -> Iterator[tuple[Cell, float]]:
        """Free 8-connected neighbours with step cost; diagonals may not cut corners."""

        grid = self.cost_map.grid
        row, column = cell
        for dr, dc, step in _NEIGHBOURS:
            neighbour = (row + dr, column + dc)
            if not grid.contains_cell(neighbour) or grid.occupied[neighbour]:
                continue
            if dr and dc and (grid.occupied[row + dr, column] or grid.occupied[row, column + dc]):
                continue
            yield neighbour, step

    def _dijkstra(self) -> tuple[np.ndarray, dict[Cell, Cell]]:
        clearance = self.cost_map.grid.clearance
        preferred = self.config.preferred_clearance_m
        weight = self.config.clearance_cost_weight
        costs = np.full(clearance.shape, math.inf)
        next_cell: dict[Cell, Cell] = {}
        costs[self.goal_cell] = 0.0
        queue = [(0.0, self.goal_cell)]
        while queue:
            cost, cell = heapq.heappop(queue)
            if cost > costs[cell]:
                continue
            for neighbour, step in self._neighbours(cell):
                # Inflation stays a hard constraint; this only makes the field
                # prefer the middle of a corridor over hugging its edge.
                deficit = max(0.0, (preferred - clearance[neighbour]) / preferred)
                candidate = cost + step * (1.0 + weight * deficit * deficit)
                if candidate + 1e-12 < costs[neighbour]:
                    costs[neighbour] = candidate
                    next_cell[neighbour] = cell
                    heapq.heappush(queue, (candidate, neighbour))
        return costs, next_cell

    def _reachable_cell(self, point: Point2D) -> Cell:
        """Nearest cell with a finite cost, for a start that drifted into inflation.

        Only the start is ever snapped, only within the safety margin, and only
        across raw-free ground.  A blocked goal is rejected, never moved.
        """

        grid = self.cost_map.grid
        cell = grid.world_to_grid(point)
        if not grid.contains_cell(cell):
            raise PlanningError(PlanningErrorCode.OUT_OF_BOUNDS, "start is outside the map")
        if math.isfinite(self.costs[cell]):
            return cell

        reach = math.ceil(self.config.safety_margin_m / grid.resolution_m) + 1
        candidates = [
            (row, column)
            for row in range(cell[0] - reach, cell[0] + reach + 1)
            for column in range(cell[1] - reach, cell[1] + reach + 1)
            if grid.contains_cell((row, column))
            and math.isfinite(self.costs[row, column])
            and all(
                grid.is_raw_free(sample)
                for sample in grid.line_samples(point, grid.grid_to_world((row, column)))
            )
        ]
        if not candidates:
            code = PlanningErrorCode.START_BLOCKED if grid.occupied[cell] else PlanningErrorCode.NO_PATH
            raise PlanningError(code, f"no collision-free path to goal ({code})")
        return min(
            candidates,
            key=lambda item: (math.hypot(item[0] - cell[0], item[1] - cell[1]), item),
        )

    def cost_to_go(self, point: Point2D) -> float:
        grid = self.cost_map.grid
        cell = grid.world_to_grid(point)
        return float(self.costs[cell]) if grid.contains_cell(cell) else math.inf

    def progress_m(self, start: Point2D, end: Point2D) -> float:
        """Cost-to-go removed by moving start->end; +/-inf when an end is off the field."""

        before, after = self.cost_to_go(start), self.cost_to_go(end)
        if math.isfinite(before) and math.isfinite(after):
            return (before - after) * self.cost_map.grid.resolution_m
        return math.inf if math.isfinite(after) else -math.inf

    def plan(self, start: Point2D) -> GlobalPlan:
        grid = self.cost_map.grid
        first = self._reachable_cell(start)
        cells = [first]
        while cells[-1] != self.goal_cell:
            cells.append(self.next_cell[cells[-1]])
        raw = [grid.grid_to_world(cell) for cell in cells]
        # Start and goal are real coordinates, not cell centres.  A snapped
        # start keeps its cell as well, so the step back out of inflation
        # stays in the path instead of being smoothed away.
        if first == grid.world_to_grid(start):
            raw[0] = start
        else:
            raw.insert(0, start)
        raw[-1] = self.goal
        path = resample_path(self._smooth(raw), self.config.waypoint_spacing_m)
        return GlobalPlan(tuple(raw), path, path_length(path))

    def _smooth(self, path: Sequence[Point2D]) -> tuple[Point2D, ...]:
        """Greedy line-of-sight shortcuts that never reduce clearance."""

        smoothed = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            target = len(path) - 1
            while target > anchor + 1 and not self._shortcut_keeps_clearance(path[anchor : target + 1]):
                target -= 1
            smoothed.append(path[target])
            anchor = target
        return tuple(smoothed)

    def _shortcut_keeps_clearance(self, segment: Sequence[Point2D]) -> bool:
        """A straight line replacing ``segment`` stays free and no closer to obstacles."""

        grid = self.cost_map.grid
        samples = list(grid.line_samples(segment[0], segment[-1]))
        return all(grid.is_free(point) for point in samples) and (
            min(grid.clearance_at(point) for point in samples) + 1e-9
            >= min(grid.clearance_at(point) for point in segment)
        )
