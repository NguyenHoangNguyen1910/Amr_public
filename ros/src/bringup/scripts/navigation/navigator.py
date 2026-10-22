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
    NavigationInput,
    NavigationResult,
    Point2D,
    Pose2D,
    State,
    SensorDataError,
    distance,
    normalize_angle,
    transform_points,
    target_point,
)

_MINIMUM_YAW_PROGRESS_RAD = 0.02


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
        self.path: tuple[Point2D, ...] = ()
        self.phase = GoalPhase.TRACKING
        self.state = State.ERROR
        self.command = STOP
        self.anchor: Point2D | None = None
        self.yaw_progress_anchor: float | None = None
        self.yaw_progress_mode: str | None = None
        self.tracking_yaw_target: Point2D | None = None
        self.tracking_yaw_best: tuple[float, float] | None = None
        self.last_progress = 0.0
        self.stable_cycles = 0
        self.clearance_m: float | None = None
        self.blocked_reason: str | None = None

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
        self.local_planner.reset_terminal()
        self.blocked_reason = None
        self.goal = goal
        self.planner = GlobalPlanner(self.cost_map, goal, self.config)
        self.path = self.planner.plan(navigation_input.pose.to_point())
        self.phase = GoalPhase.TRACKING
        self.state = State.NAVIGATING
        self.command = STOP
        self.anchor = navigation_input.pose.to_point()
        self.yaw_progress_anchor = None
        self.yaw_progress_mode = None
        self.tracking_yaw_target = None
        self.tracking_yaw_best = None
        self.last_progress = time.monotonic() if now is None else now
        self.stable_cycles = 0
        self.clearance_m = None

    def cancel(self) -> None:
        self.command = STOP
        self.state = State.CANCELLED

    def update_map(self, static_map: OccupancyGrid, now: float | None = None) -> None:
        """Apply a new SLAM map without restarting the mission or its watchdog."""
        if self.goal is None:
            raise RuntimeError("plan() must be called before update_map()")
        self.command = STOP
        self.cost_map.replace_static(static_map, time.monotonic() if now is None else now)
        try:
            self.planner = GlobalPlanner(self.cost_map, self.goal, self.config)
        except PlanningError:
            self.state = State.BLOCKED
            raise

    def _tracking_yaw_progress(self, pose: Pose2D) -> bool:
        # Freeze the bearing until actual translation makes progress. A changing
        # path or an oscillating heading must not keep a stuck robot alive forever.
        if self.tracking_yaw_target is None:
            self.tracking_yaw_target = target_point(
                pose.to_point(), self.path, self.config.lookahead_distance_m
            )
        target = self.tracking_yaw_target
        bearing = math.atan2(target.y - pose.y, target.x - pose.x)
        errors = (
            abs(normalize_angle(bearing - pose.yaw)),
            abs(normalize_angle(bearing - pose.yaw - math.pi)),
        )
        if self.tracking_yaw_best is None:
            self.tracking_yaw_best = errors
            return False
        best = self.tracking_yaw_best
        improved = tuple(old - new >= _MINIMUM_YAW_PROGRESS_RAD for old, new in zip(best, errors))
        self.tracking_yaw_best = tuple(new if ok else old for old, new, ok in zip(best, errors, improved))
        return any(improved)

    def _final_phase(self, navigation_input: NavigationInput) -> None:
        if self.goal is None:
            return
        distance_to_goal = distance(navigation_input.pose.to_point(), self.goal)
        if (
            self.phase is GoalPhase.TRACKING
            and distance_to_goal <= self.config.goal_terminal_entry_m
        ):
            self.phase = GoalPhase.FINAL
            yaw_progress = self._terminal_yaw_error(navigation_input.pose)
            if yaw_progress is not None:
                self.yaw_progress_mode, self.yaw_progress_anchor = yaw_progress
        elif (
            self.phase is GoalPhase.FINAL
            and distance_to_goal > self.config.goal_terminal_exit_m
        ):
            self.phase = GoalPhase.TRACKING
            self.yaw_progress_anchor = None
            self.yaw_progress_mode = None

    def _terminal_yaw_error(self, pose: Pose2D) -> tuple[str, float] | None:
        if self.goal is None:
            return None
        position = pose.to_point()
        if (distance(position, self.goal) > self.config.goal_tolerance_m
                and not self.local_planner.aligning_goal_yaw):
            target_yaw = math.atan2(self.goal.y - pose.y, self.goal.x - pose.x)
            mode = "bearing"
        elif self.goal.yaw is not None:
            target_yaw = self.goal.yaw
            mode = "goal"
        else:
            return None
        return mode, abs(normalize_angle(target_yaw - pose.yaw))

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

    def _result(self, pose) -> NavigationResult:
        if self.goal is None or not self.path:
            raise RuntimeError("navigation session is not planned")
        return NavigationResult(
            self.state,
            self.command,
            pose,
            self.goal,
            self.path,
            self.clearance_m,
        )

    def _record_progress(self, pose: Pose2D, position: Point2D, now: float) -> None:
        if self.anchor is not None:
            headway = (
                self.planner.progress_m(self.anchor, position)
                >= self.config.minimum_progress_m
            )
            if headway:
                self.anchor = position
                self.last_progress = now
                self.tracking_yaw_target = None
                self.tracking_yaw_best = None
        if self.phase is GoalPhase.TRACKING and self._tracking_yaw_progress(pose):
            self.last_progress = now
        if self.phase is GoalPhase.FINAL:
            yaw_progress = self._terminal_yaw_error(pose)
            if yaw_progress is None:
                self.yaw_progress_anchor = None
                self.yaw_progress_mode = None
            else:
                yaw_mode, yaw_error = yaw_progress
                if self.yaw_progress_mode != yaw_mode:
                    self.yaw_progress_mode = yaw_mode
                    self.yaw_progress_anchor = yaw_error
                elif (
                    self.yaw_progress_anchor is not None
                    and self.yaw_progress_anchor - yaw_error >= _MINIMUM_YAW_PROGRESS_RAD
                ):
                    self.yaw_progress_anchor = yaw_error
                    self.last_progress = now

    def _update_state(self, navigation_input: NavigationInput, safe: bool, now: float) -> None:
        if now - self.last_progress > self.config.no_progress_timeout_seconds:
            self.command = STOP
            self.state = State.BLOCKED
            if self.blocked_reason is None:
                self.blocked_reason = "NO_PROGRESS: measured pose did not make sufficient progress"

        if (self.state is State.NAVIGATING and safe
                and self.phase is GoalPhase.FINAL and self._goal_is_stable(navigation_input)):
            self.stable_cycles += 1
            if self.stable_cycles >= self.config.stable_goal_cycles:
                self.command = STOP
                self.state = State.SUCCEEDED
        else:
            self.stable_cycles = 0

    def step(
        self, navigation_input: NavigationInput, now: float | None = None
    ) -> NavigationResult:
        if self.goal is None or self.planner is None or not self.path:
            raise RuntimeError("plan() must be called before step()")
        if self.state is not State.NAVIGATING:
            return self._result(navigation_input.pose)

        now = time.monotonic() if now is None else now
        self._final_phase(navigation_input)
        position = navigation_input.pose.to_point()
        # Inflation describes command safety, not whether an observation exists.
        # Dropping scans inside inflation lets a still-visible obstacle expire,
        # producing alternating move/stop commands at the obstacle TTL.
        seen = transform_points(navigation_input.pose, navigation_input.obstacles_robot)
        try:
            self.cost_map.update_world_obstacles(seen, now)
            self.planner.refresh()
            self.path = self.planner.plan(position)
            decision = self.local_planner.command(
                navigation_input.pose,
                navigation_input.velocity,
                self.path,
                navigation_input.obstacles_robot,
                self.planner,
                self.goal,
                self.phase is GoalPhase.FINAL,
            )
        except (PlanningError, SensorDataError) as error:
            self.command = STOP
            self.clearance_m = None
            self.stable_cycles = 0
            self.blocked_reason = (
                f"{error.code.value}: {error}" if isinstance(error, PlanningError)
                else f"SENSOR_DATA: {error}"
            )
            if now - self.last_progress > self.config.no_progress_timeout_seconds:
                self.state = State.BLOCKED
            return self._result(navigation_input.pose)

        self.command = decision.command if decision.safe else STOP
        self.blocked_reason = None if decision.safe else "UNSAFE_COMMAND: local collision check rejected motion"
        self.clearance_m = decision.clearance_m
        self._record_progress(navigation_input.pose, position, now)
        self._update_state(navigation_input, decision.safe, now)
        return self._result(navigation_input.pose)
