"""Occupancy grid, dynamic costmap, and global path planning."""

from __future__ import annotations

import heapq
import math
from collections.abc import Iterator, Sequence
from enum import Enum

import numpy as np
from scipy.ndimage import distance_transform_edt

from .config import NavigationConfig
from .types import GlobalPlan, Point2D, distance, lerp, resample_path

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


class OccupancyGrid:
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
        self.clearance = (
            distance_transform_edt(~raw, sampling=resolution_m)
            if raw.any()
            else np.full(raw.shape, math.inf)
        )
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
        cell = self.world_to_grid(point)
        return self.contains_cell(cell) and not bool(self.occupied[cell])

    def is_raw_free(self, point: Point2D) -> bool:
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


class CostMap:
    """Static occupancy plus temporary obstacles observed by LiDAR."""

    def __init__(self, static: OccupancyGrid, obstacle_ttl_seconds: float):
        self.static = static
        self.grid = static
        self.obstacle_ttl_seconds = obstacle_ttl_seconds
        self._expires_at = np.zeros(static.raw.shape)
        self._dynamic = np.zeros(static.raw.shape, dtype=np.bool_)
        self.version = 0

    def update_world_obstacles(self, points: Sequence[Point2D], now: float) -> None:
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
        self.grid = self.static
        if dynamic.any():
            self.grid = OccupancyGrid(
                self.static.raw | dynamic,
                min_x=self.static.min_x,
                min_y=self.static.min_y,
                resolution_m=self.static.resolution_m,
                inflation_radius_m=self.static.inflation_radius,
            )
        self.version += 1

    def is_free(self, point: Point2D) -> bool:
        return self.grid.is_free(point)

    def is_raw_free(self, point: Point2D) -> bool:
        return self.grid.is_raw_free(point)

    def clearance_at(self, point: Point2D) -> float:
        return self.grid.clearance_at(point)

    def static_is_free(self, point: Point2D) -> bool:
        return self.static.is_free(point)


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
    """Dijkstra cost-to-go field and path for one goal."""

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
        if self.cost_map.grid.occupied[self.goal_cell]:
            raise PlanningError(
                PlanningErrorCode.GOAL_BLOCKED, "goal is in an inflated obstacle"
            )
        if self._version != self.cost_map.version:
            self._build_cost_field()
            self._version = self.cost_map.version

    def _neighbours(self, cell: Cell) -> Iterator[tuple[Cell, float]]:
        grid = self.cost_map.grid
        row, column = cell
        for row_delta, column_delta, step in _NEIGHBOURS:
            neighbour = (row + row_delta, column + column_delta)
            if not grid.contains_cell(neighbour) or grid.occupied[neighbour]:
                continue
            if row_delta and column_delta and (
                grid.occupied[row + row_delta, column]
                or grid.occupied[row, column + column_delta]
            ):
                continue
            yield neighbour, step

    def _build_cost_field(self) -> None:
        self.costs = np.full(self.cost_map.grid.raw.shape, math.inf)
        self.next_cell = {}
        self.costs[self.goal_cell] = 0.0
        queue = [(0.0, self.goal_cell)]

        while queue:
            cost, cell = heapq.heappop(queue)
            if cost > self.costs[cell]:
                continue
            for neighbour, step in self._neighbours(cell):
                candidate = cost + step
                if candidate + 1e-12 < self.costs[neighbour]:
                    self.costs[neighbour] = candidate
                    self.next_cell[neighbour] = cell
                    heapq.heappush(queue, (candidate, neighbour))

    def _reachable_cell(self, point: Point2D) -> Cell:
        grid = self.cost_map.grid
        cell = grid.world_to_grid(point)
        if not grid.contains_cell(cell):
            raise PlanningError(PlanningErrorCode.OUT_OF_BOUNDS, "start is outside the map")
        if math.isfinite(self.costs[cell]):
            return cell

        reach = math.ceil(self.config.safety_margin_m / grid.resolution_m) + 1
        candidates = (
            (row, column)
            for row in range(cell[0] - reach, cell[0] + reach + 1)
            for column in range(cell[1] - reach, cell[1] + reach + 1)
            if grid.contains_cell((row, column))
            and math.isfinite(self.costs[row, column])
            and all(
                grid.is_raw_free(sample)
                for sample in grid.line_samples(point, grid.grid_to_world((row, column)))
            )
        )
        try:
            return min(candidates, key=lambda item: (math.dist(item, cell), item))
        except ValueError:
            code = (
                PlanningErrorCode.START_BLOCKED
                if grid.occupied[cell]
                else PlanningErrorCode.NO_PATH
            )
            raise PlanningError(code, f"no collision-free path to goal ({code})") from None

    def cost_to_go(self, point: Point2D) -> float:
        cell = self.cost_map.grid.world_to_grid(point)
        return (
            float(self.costs[cell])
            if self.cost_map.grid.contains_cell(cell)
            else math.inf
        )

    def progress_m(self, start: Point2D, end: Point2D) -> float:
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
        if first == grid.world_to_grid(start):
            raw[0] = start
        else:
            raw.insert(0, start)
        raw[-1] = self.goal
        path = resample_path(self._smooth(raw), self.config.waypoint_spacing_m)
        return GlobalPlan(path)

    def _smooth(self, path: Sequence[Point2D]) -> tuple[Point2D, ...]:
        grid = self.cost_map.grid
        result = [path[0]]
        anchor = 0
        while anchor < len(path) - 1:
            target = len(path) - 1
            while target > anchor + 1 and not grid.line_is_free(
                path[anchor], path[target]
            ):
                target -= 1
            result.append(path[target])
            anchor = target
        return tuple(result)
