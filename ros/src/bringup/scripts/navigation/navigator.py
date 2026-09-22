"""Synchronous navigation session joining global and local planning."""

from __future__ import annotations

import math
import time

from .config import DEFAULT_CONFIG, NavigationConfig
from .global_planner import CostMap, GlobalPlanner, OccupancyGrid, PlanningError
from .local_planner import LocalPlanner
from .types import (
    STOP,
    GoalPhase,
    GlobalPlan,
    NavigationInput,
    NavigationResult,
    Point2D,
    State,
    Velocity,
    SensorDataError,
    distance,
    normalize_angle,
    transform_points,
)


class Navigator:
    """One goal session; callers drive it by calling :meth:`step`."""

    def __init__(
        self,
        static_map: OccupancyGrid,
        config: NavigationConfig = DEFAULT_CONFIG,
    ):
        self.config = config
        self.cost_map = CostMap(static_map, config.temporary_obstacle_ttl_seconds)
        self.local_planner = LocalPlanner(self.cost_map, config)
        self.goal: Point2D | None = None
        self.planner: GlobalPlanner | None = None
        self.plan_data: GlobalPlan | None = None
        self.phase = GoalPhase.TRACKING
        self.state = State.ERROR
        self.command = STOP
        self.anchor: Point2D | None = None
        self.last_progress = 0.0
        self.stable_cycles = 0
        self.clearance_m: float | None = None
        self.error: str | None = None

    def plan(
        self,
        goal: Point2D,
        navigation_input: NavigationInput,
        now: float | None = None,
    ) -> None:
        if not (math.isfinite(goal.x) and math.isfinite(goal.y)):
            raise ValueError("goal coordinates must be finite")
        if goal.yaw is not None and not math.isfinite(goal.yaw):
            raise ValueError("goal yaw must be finite")
        self.cost_map.clear_dynamic_obstacles()
        self.goal = goal
        self.planner = GlobalPlanner(self.cost_map, goal, self.config)
        self.plan_data = self.planner.plan(navigation_input.pose.to_point())
        self.phase = GoalPhase.TRACKING
        self.state = State.NAVIGATING
        self.command = STOP
        self.anchor = navigation_input.pose.to_point()
        self.last_progress = time.monotonic() if now is None else now
        self.stable_cycles = 0
        self.clearance_m = None
        self.error = None

    def cancel(self) -> None:
        self.command = STOP
        self.state = State.CANCELLED

    def _final_phase(self, navigation_input: NavigationInput) -> None:
        if self.goal is None:
            return
        distance_to_goal = distance(navigation_input.pose.to_point(), self.goal)
        if (
            self.phase is GoalPhase.TRACKING
            and distance_to_goal <= self.config.goal_terminal_entry_m
        ):
            self.phase = GoalPhase.FINAL
        elif self.phase is GoalPhase.FINAL and distance_to_goal > self.config.goal_terminal_exit_m:
            self.phase = GoalPhase.TRACKING

    def _goal_is_stable(self, navigation_input: NavigationInput) -> bool:
        if self.goal is None:
            return False
        pose = navigation_input.pose
        position_ok = distance(pose.to_point(), self.goal) <= self.config.goal_tolerance_m
        yaw_ok = self.goal.yaw is None or (
            abs(normalize_angle(pose.yaw - self.goal.yaw))
            <= self.config.goal_yaw_tolerance_rad
        )
        velocity_ok = (
            abs(navigation_input.velocity.linear) <= self.config.stable_linear_velocity_mps
            and abs(navigation_input.velocity.angular) <= self.config.stable_angular_velocity_rps
        )
        return position_ok and yaw_ok and velocity_ok

    def _result(self, pose, error: str | None = None) -> NavigationResult:
        if self.goal is None or self.plan_data is None:
            raise RuntimeError("navigation session is not planned")
        return NavigationResult(
            self.state,
            self.command,
            pose,
            self.goal,
            self.plan_data.path,
            self.clearance_m,
            error or self.error,
        )

    def step(
        self, navigation_input: NavigationInput, now: float | None = None
    ) -> NavigationResult:
        if self.goal is None or self.planner is None or self.plan_data is None:
            raise RuntimeError("plan() must be called before step()")
        if self.state is not State.NAVIGATING:
            return self._result(navigation_input.pose)

        now = time.monotonic() if now is None else now
        self._final_phase(navigation_input)
        position = navigation_input.pose.to_point()
        seen = (
            transform_points(navigation_input.pose, navigation_input.obstacles_robot)
            if self.cost_map.is_free(position)
            else ()
        )
        try:
            self.cost_map.update_world_obstacles(seen, now)
            self.planner.refresh()
            self.plan_data = self.planner.plan(position)
            decision = self.local_planner.command(
                navigation_input.pose,
                navigation_input.velocity,
                self.plan_data.path,
                navigation_input.obstacles_robot,
                self.planner,
                self.goal,
                self.phase is GoalPhase.FINAL,
            )
        except (PlanningError, SensorDataError) as error:
            self.command = STOP
            self.clearance_m = None
            self.error = str(error)
            if now - self.last_progress > self.config.no_progress_timeout_seconds:
                self.state = State.BLOCKED
            return self._result(navigation_input.pose)

        self.command = decision.command if decision.safe else STOP
        self.clearance_m = decision.clearance_m
        if self.anchor is not None:
            headway = (
                self.planner.progress_m(self.anchor, position)
                >= self.config.minimum_progress_m
            )
            if headway:
                self.anchor = position
        else:
            headway = False
        if (
            headway
            or decision.escaping
            or abs(self.command.angular) > 0.05
            or self.command != STOP
        ):
            self.last_progress = now
        if now - self.last_progress > self.config.no_progress_timeout_seconds:
            self.command = STOP
            self.state = State.BLOCKED
            self.error = "no progress"

        if self.phase is GoalPhase.FINAL and self._goal_is_stable(navigation_input):
            self.stable_cycles += 1
            if self.stable_cycles >= self.config.stable_goal_cycles:
                self.command = STOP
                self.state = State.SUCCEEDED
        else:
            self.stable_cycles = 0
        return self._result(navigation_input.pose)
