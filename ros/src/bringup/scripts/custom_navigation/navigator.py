"""Threaded navigation state machine joining global and local planning."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol, TypeVar

from .config import DEFAULT_CONFIG, NavigationConfig
from .global_planner import (
    CostMap,
    GlobalPlanner,
    OccupancyGrid,
    PlanningError,
    PlanningErrorCode,
)
from .helpers import (
    SensorDataError,
    distance,
    normalize_angle,
    transform_points,
)
from .local_planner import LocalPlanner
from .types import (
    STOP,
    ControlDecision,
    DecisionReason,
    GlobalPlan,
    GoalPhase,
    NavigationInput,
    NavigationStatus,
    Point2D,
    Pose2D,
    State,
    Velocity,
)

T = TypeVar("T")

_MESSAGES = {
    State.IDLE: "Select goal",
    State.PLANNING: "Planning",
    State.READY: "Ready",
    State.NAVIGATING: "Moving",
    State.RESETTING: "Resetting",
    State.SUCCEEDED: "Goal reached",
    State.CANCELLED: "Stopped",
    State.BLOCKED: "Blocked",
    State.ERROR: "Error",
}
_AVOIDING = "Avoiding obstacle"
_PHASE_MESSAGES = {
    GoalPhase.TRACKING: "Moving",
    GoalPhase.SETTLING_POSITION: "Settling position",
    GoalPhase.SETTLING_YAW: "Aligning yaw",
    GoalPhase.VERIFYING: "Verifying goal",
}
# States in which an autonomous mission owns the velocity command.
_CONTROL_STATES = frozenset({State.PLANNING, State.READY, State.NAVIGATING})
# States in which a new plan request has to be refused.
_BUSY_STATES = frozenset({State.PLANNING, State.NAVIGATING, State.RESETTING})


class NavigationConflict(RuntimeError):
    pass


class NavigationTransport(Protocol):
    def navigation_input(self) -> NavigationInput: ...

    def set_velocity(self, linear: float, angular: float) -> object: ...

    def reset(self) -> bool: ...


def _require_finite(goal: Point2D) -> None:
    if not (math.isfinite(goal.x) and math.isfinite(goal.y)):
        raise ValueError("goal coordinates must be finite")
    if goal.yaw is not None and not math.isfinite(goal.yaw):
        raise ValueError("goal yaw must be finite")


@dataclass
class _Mission:
    """Mutable state for one autonomous mission; a new instance per plan/reset."""

    state: State = State.IDLE
    message: str = _MESSAGES[State.IDLE]
    error: str | None = None
    start: Point2D | None = None
    goal: Point2D | None = None
    position: Point2D | None = None
    planner: GlobalPlanner | None = None
    plan: GlobalPlan | None = None
    command: Velocity = STOP
    clearance_m: float | None = None
    anchor: Point2D | None = None
    last_progress: float = 0.0
    last_scan: int = 0
    phase: GoalPhase = GoalPhase.TRACKING
    current_pose: Pose2D | None = None
    velocity: Velocity = STOP
    stable_cycles: int = 0


@dataclass(frozen=True)
class _Cycle:
    """Mission state read once at the top of a control cycle.

    Carrying it forward lets the algorithms run unlocked; ``revision`` is what
    the commit step checks to know the mission has not moved on meanwhile.
    """

    revision: int
    planner: GlobalPlanner
    previous: Velocity
    anchor: Point2D
    goal: Point2D
    phase: GoalPhase
    goal_yaw: float | None


class CustomNavigator:
    """Own autonomous command publication and expose a JSON-safe status.

    Every cycle: fold LiDAR into the costmap, rebuild the cost-to-go field if
    the costmap changed, read the path from the robot's cell, publish a command.

    Locking: methods named ``*_locked`` require ``_lock`` to already be held by
    the caller; every other method takes it itself.  Long algorithms run under
    ``_algorithm_lock`` with ``_lock`` released, so mission controls stay
    responsive, and commit only if the mission revision is unchanged.
    """

    def __init__(
        self,
        transport: NavigationTransport,
        *,
        static_map: OccupancyGrid | None = None,
        config: NavigationConfig = DEFAULT_CONFIG,
        clock: Callable[[], float] = time.monotonic,
        auto_start: bool = True,
    ):
        self.transport = transport
        self.config = config
        if static_map is None:
            raise ValueError("a ROS occupancy map is required")
        self.map = static_map
        self.cost_map = CostMap(self.map, config.temporary_obstacle_ttl_seconds)
        self.local_planner = LocalPlanner(self.cost_map, config)
        self.clock = clock
        self._lock = threading.Lock()
        self._algorithm_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._closed = False
        self._revision = 0
        self._mission = _Mission(last_progress=self.clock())
        self._thread = threading.Thread(target=self._loop, name="amr-navigation", daemon=True)
        if auto_start:
            self._thread.start()

    @property
    def state(self) -> State:
        with self._lock:
            return self._mission.state

    @property
    def owns_control(self) -> bool:
        with self._lock:
            return self._mission.state in _CONTROL_STATES

    # -- public mission API -----------------------------------------------------

    def plan(self, goal: Point2D) -> NavigationStatus:
        _require_finite(goal)
        with self._lock:
            if self._mission.state in _BUSY_STATES:
                raise NavigationConflict("navigation mission is active")
            revision = self._begin_locked(State.PLANNING)

        def work() -> tuple[NavigationInput, GlobalPlanner, GlobalPlan]:
            nav_input = self.transport.navigation_input()
            start = nav_input.pose.to_point()
            self.cost_map.clear_dynamic_obstacles()
            planner = GlobalPlanner(self.cost_map, goal, self.config)
            return nav_input, planner, planner.plan(start)

        nav_input, planner, plan = self._run_unlocked(revision, work, "Plan failed")
        with self._lock:
            if revision != self._revision:
                raise NavigationConflict("planning was cancelled")
            start = nav_input.pose.to_point()
            self._mission = _Mission(
                state=State.READY,
                message=_MESSAGES[State.READY],
                start=start,
                goal=goal,
                position=start,
                planner=planner,
                plan=plan,
                anchor=start,
                last_progress=self.clock(),
                current_pose=nav_input.pose,
                velocity=nav_input.velocity,
            )
            return self._status_locked()

    def is_goal_free(self, goal: Point2D) -> bool:
        """Whether a goal is inside the map and clear of inflated obstacles."""

        _require_finite(goal)
        return self.map.is_free(goal)

    def start(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state != State.READY:
                raise NavigationConflict("a ready plan is required")
            self._set_state_locked(State.NAVIGATING)
            self._mission.last_progress = self.clock()
            return self._status_locked()

    def cancel(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state == State.RESETTING:
                raise NavigationConflict("reset is in progress")
            self._begin_locked(State.CANCELLED)
            self._mission.clearance_m = None
            return self._status_locked()

    def set_speed_mps(self, speed_mps: float) -> NavigationStatus:
        """Change the DWA forward-speed cap without restarting a mission."""

        with self._lock:
            self.local_planner.set_speed_mps(speed_mps)
            return self._status_locked()

    def reset(self) -> NavigationStatus:
        with self._lock:
            if self._mission.state == State.RESETTING:
                raise NavigationConflict("reset is already in progress")
            revision = self._begin_locked(State.RESETTING)

        def work() -> bool:
            reset_ok = self.transport.reset()
            self.cost_map.clear_dynamic_obstacles()
            return reset_ok

        reset_ok = self._run_unlocked(revision, work, "Reset failed")
        with self._lock:
            if revision == self._revision:
                self._mission = (
                    _Mission(last_progress=self.clock())
                    if reset_ok
                    else _Mission(
                        state=State.ERROR,
                        message="Reset failed",
                        error="Gazebo reset failed",
                        last_progress=self.clock(),
                    )
                )
            return self._status_locked()

    def status(self) -> NavigationStatus:
        with self._lock:
            return self._status_locked()

    # -- mission state, caller holds the lock -----------------------------------

    def _begin_locked(
        self, state: State, *, error: str | None = None, message: str | None = None
    ) -> int:
        """Invalidate in-flight work, stop the robot, and enter ``state``."""

        self._revision += 1
        self._stop_locked()
        self._set_state_locked(state, error=error, message=message)
        return self._revision

    def _run_unlocked(self, revision: int, work: Callable[[], T], failure: str) -> T:
        """Run ``work`` under the algorithm lock only; mark ERROR if it raises."""

        try:
            with self._algorithm_lock:
                return work()
        except Exception as error:
            with self._lock:
                if revision == self._revision:
                    self._set_state_locked(State.ERROR, error=str(error), message=failure)
            raise

    def _set_state_locked(
        self, state: State, *, error: str | None = None, message: str | None = None
    ) -> None:
        mission = self._mission
        mission.state = state
        mission.message = message or _MESSAGES[state]
        mission.error = error

    def _stop_locked(self) -> None:
        self._mission.command = STOP
        try:
            self.transport.set_velocity(0.0, 0.0)
        except RuntimeError:
            # The transport refuses commands mid-reset; it is already stopped.
            pass

    def _block_if_stalled_locked(self, now: float, error: str) -> None:
        if now - self._mission.last_progress > self.config.no_progress_timeout_seconds:
            self._stop_locked()
            self._set_state_locked(State.BLOCKED, error=error)

    def _status_locked(self) -> NavigationStatus:
        mission = self._mission
        pose, goal, plan = mission.position, mission.goal, mission.plan
        return {
            "state": mission.state.value,
            "message": mission.message,
            "phase": mission.phase.value,
            "owns_control": mission.state in _CONTROL_STATES,
            "start": None if mission.start is None else mission.start.to_dict(),
            "goal": None if goal is None else goal.to_dict(),
            "distance_to_goal_m": None if pose is None or goal is None else distance(pose, goal),
            "yaw_error_to_goal_rad": (
                None
                if mission.current_pose is None or goal is None or goal.yaw is None
                else abs(normalize_angle(mission.current_pose.yaw - goal.yaw))
            ),
            "current_pose": (
                None
                if mission.current_pose is None
                else {
                    "x": mission.current_pose.x,
                    "y": mission.current_pose.y,
                    "yaw": mission.current_pose.yaw,
                }
            ),
            "measured_velocity": mission.velocity.to_dict(),
            "path_length_m": None if plan is None else plan.length_m,
            "waypoint_count": 0 if plan is None else len(plan.path),
            "global_path": [] if plan is None else [point.to_dict() for point in plan.path],
            "command": mission.command.to_dict(),
            "speed_mps": self.local_planner.speed_mps,
            "minimum_clearance_m": mission.clearance_m,
            "error": mission.error,
        }

    # -- control thread ---------------------------------------------------------

    def _step(self) -> None:
        """One control cycle: read sensors, choose a command, publish it."""

        nav_input = self.transport.navigation_input()
        cycle = self._begin_cycle(nav_input)
        if cycle is None:
            return
        now = self.clock()
        with self._algorithm_lock:
            try:
                plan, decision = self._decide(nav_input, cycle, now)
            except PlanningError as error:
                self._handle_no_path(cycle.revision, now, error.code)
                return
        self._commit(cycle, nav_input, plan, decision, now)

    def _update_phase_locked(self, nav_input: NavigationInput) -> None:
        """Advance the terminal controller with hysteresis and stable checks."""

        mission = self._mission
        if mission.goal is None:
            raise RuntimeError("active navigation goal is incomplete")
        position = nav_input.pose.to_point()
        position_error = distance(position, mission.goal)
        yaw_error = (
            0.0
            if mission.goal.yaw is None
            else abs(normalize_angle(nav_input.pose.yaw - mission.goal.yaw))
        )
        position_reached = position_error <= self.config.goal_tolerance_m
        yaw_reached = mission.goal.yaw is None or yaw_error <= self.config.goal_yaw_tolerance_rad
        velocity_stable = (
            abs(nav_input.velocity.linear) <= self.config.stable_linear_velocity_mps
            and abs(nav_input.velocity.angular) <= self.config.stable_angular_velocity_rps
        )

        previous = mission.phase
        if mission.phase is GoalPhase.TRACKING:
            if position_error <= self.config.goal_terminal_entry_m:
                mission.phase = GoalPhase.SETTLING_POSITION
        if mission.phase is GoalPhase.SETTLING_POSITION:
            if position_error > self.config.goal_terminal_exit_m:
                mission.phase = GoalPhase.TRACKING
            elif position_reached and velocity_stable:
                mission.phase = (
                    GoalPhase.SETTLING_YAW
                    if not yaw_reached
                    else GoalPhase.VERIFYING
                )
        elif mission.phase is GoalPhase.SETTLING_YAW:
            if position_error > self.config.goal_terminal_exit_m:
                mission.phase = GoalPhase.SETTLING_POSITION
            elif yaw_reached and position_reached and velocity_stable:
                mission.phase = GoalPhase.VERIFYING
        elif mission.phase is GoalPhase.VERIFYING:
            if position_error > self.config.goal_terminal_exit_m:
                mission.phase = GoalPhase.SETTLING_POSITION
            elif not yaw_reached:
                mission.phase = GoalPhase.SETTLING_YAW

        if mission.phase is not GoalPhase.VERIFYING:
            mission.stable_cycles = 0
        elif (
            position_reached
            and yaw_reached
            and velocity_stable
        ):
            mission.stable_cycles += 1
        else:
            mission.stable_cycles = 0

        if mission.phase is not previous:
            mission.stable_cycles = 0

        if (
            mission.phase is GoalPhase.VERIFYING
            and mission.stable_cycles >= self.config.stable_goal_cycles
        ):
            self._stop_locked()
            self._set_state_locked(State.SUCCEEDED)

    def _begin_cycle(self, nav_input: NavigationInput) -> _Cycle | None:
        """Record the new pose and decide whether this cycle needs a decision.

        Returns ``None`` when the mission ended, the goal was reached, or the
        scan has not advanced -- the last two still publish a command.
        """

        position = nav_input.pose.to_point()
        with self._lock:
            mission = self._mission
            mission.position = position
            mission.current_pose = nav_input.pose
            mission.velocity = nav_input.velocity
            if mission.state != State.NAVIGATING:
                return None
            if mission.goal is None or mission.planner is None or mission.anchor is None:
                raise RuntimeError("active navigation plan is incomplete")
            self._update_phase_locked(nav_input)
            if mission.state is State.SUCCEEDED:
                return None
            if (
                mission.phase is GoalPhase.TRACKING
                and nav_input.scan_sequence
                and nav_input.scan_sequence == mission.last_scan
            ):
                # Same scan as last cycle: re-publish, do not re-decide.
                self.transport.set_velocity(mission.command.linear, mission.command.angular)
                return None
            mission.last_scan = nav_input.scan_sequence
            return _Cycle(
                self._revision,
                mission.planner,
                mission.command,
                mission.anchor,
                mission.goal,
                mission.phase,
                mission.goal.yaw,
            )

    def _decide(
        self, nav_input: NavigationInput, cycle: _Cycle, now: float
    ) -> tuple[GlobalPlan, ControlDecision]:
        """Fold the scan into the costmap, re-read the path, pick a command."""

        pose = nav_input.pose
        position = pose.to_point()
        # Returns taken from inside inflation are the robot's own surroundings,
        # not a new obstacle; folding them in would wall the robot off.
        seen = (
            transform_points(pose, nav_input.obstacles_robot)
            if self.cost_map.is_free(position)
            else ()
        )
        self.cost_map.update_world_obstacles(seen, now)
        cycle.planner.refresh()
        plan = cycle.planner.plan(position)
        decision = self.local_planner.command(
            pose=pose,
            velocity=nav_input.velocity,
            path_world=plan.path,
            obstacles_robot=nav_input.obstacles_robot,
            previous=cycle.previous,
            progress_field=cycle.planner,
            goal_yaw=cycle.goal_yaw,
            phase=cycle.phase,
            goal=cycle.goal,
        )
        return plan, decision

    def _commit(
        self,
        cycle: _Cycle,
        nav_input: NavigationInput,
        plan: GlobalPlan,
        decision: ControlDecision,
        now: float,
    ) -> None:
        """Publish the decision, unless the mission moved on while we computed."""

        with self._lock:
            mission = self._mission
            if cycle.revision != self._revision or mission.state != State.NAVIGATING:
                return
            mission.position = nav_input.pose.to_point()
            mission.current_pose = nav_input.pose
            mission.velocity = nav_input.velocity
            mission.plan = plan
            mission.clearance_m = decision.clearance_m
            mission.command = decision.command if decision.safe else STOP
            self.transport.set_velocity(mission.command.linear, mission.command.angular)
            # Headway resets the stall timer; so do backing away and turning,
            # which are active recovery rather than stalled motion.
            position = nav_input.pose.to_point()
            headway = cycle.planner.progress_m(cycle.anchor, position) >= self.config.minimum_progress_m
            if headway:
                mission.anchor = position
            if (
                headway
                or decision.reason is DecisionReason.ESCAPING
                or abs(decision.command.angular) > self.config.active_turn_rps
                or cycle.phase is not GoalPhase.VERIFYING
                and decision.command != STOP
            ):
                mission.last_progress = now
            close = (
                decision.clearance_m is not None
                and decision.clearance_m < self.config.close_obstacle_clearance_m
            )
            mission.message = _AVOIDING if close else _PHASE_MESSAGES[cycle.phase]
            self._block_if_stalled_locked(now, "no progress")

    def _handle_no_path(self, revision: int, now: float, code: PlanningErrorCode) -> None:
        """No path this cycle: stand still; marks expire, so retry until timeout."""

        with self._lock:
            mission = self._mission
            if revision != self._revision or mission.state != State.NAVIGATING:
                return
            self._stop_locked()
            mission.message = _AVOIDING
            mission.clearance_m = None
            self._block_if_stalled_locked(now, f"no collision-free path to goal ({code})")

    def _fail(self, error: Exception) -> None:
        with self._lock:
            if self._mission.state != State.NAVIGATING:
                return
            self._begin_locked(
                State.ERROR,
                error=str(error),
                message="Sensor error" if isinstance(error, SensorDataError) else None,
            )

    def _loop(self) -> None:
        period = 1.0 / self.config.heartbeat_hz
        while not self._stop_event.is_set():
            started = self.clock()
            if self.state == State.NAVIGATING:
                try:
                    self._step()
                except Exception as error:
                    self._fail(error)
            self._stop_event.wait(max(0.0, period - (self.clock() - started)))

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        with self._lock:
            self._revision += 1
            self._stop_locked()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
