"""Dynamic-window local trajectory selection."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .config import NavigationConfig
from .helpers import (
    distance,
    distance_to_segment,
    min_distance,
    normalize_angle,
    target_point,
    transform_point,
)
from .types import (
    STOP,
    ControlDecision,
    CostMapView,
    DecisionReason,
    GoalPhase,
    Point2D,
    Pose2D,
    ProgressField,
    Velocity,
)

_ORIGIN = Point2D(0.0, 0.0)


def _distance_to_path(point: Point2D, path: Sequence[Point2D]) -> float:
    if len(path) == 1:
        return distance(point, path[0])
    return min(distance_to_segment(point, start, end) for start, end in zip(path, path[1:]))


def _reported_clearance(distance_m: float, robot_radius_m: float) -> float | None:
    """Gap between the footprint and the nearest return, for status reporting."""

    return None if math.isinf(distance_m) else max(0.0, distance_m - robot_radius_m)


def _to_world(pose: Pose2D, local: Pose2D) -> Pose2D:
    point = transform_point(pose, local.to_point())
    return Pose2D(point.x, point.y, normalize_angle(pose.yaw + local.yaw))


@dataclass(frozen=True)
class _Candidate:
    """One scored arc that passed both collision checks."""

    score: float
    linear: float
    angular: float
    trajectory: tuple[Pose2D, ...]
    clearance: float
    progress: float

    @property
    def rank(self) -> tuple[float, float, float, float, float]:
        """Total order used to pick a winner; ties break towards safety."""

        return (self.score, self.clearance, self.progress, self.linear, -abs(self.angular))


class LocalPlanner:
    """Sample reachable velocity commands and select the safest best one.

    Candidate arcs are simulated only in memory.  A new command is selected
    for every fresh LiDAR scan, so successive choices can form multi-step
    manoeuvres without hard-coded spin or backup recovery sequences.
    """

    def __init__(self, cost_map: CostMapView, config: NavigationConfig):
        self.cost_map = cost_map
        self.config = config
        self._linear_velocity_cap = config.default_linear_velocity_mps

    @property
    def speed_mps(self) -> float:
        """Current forward-speed cap selected by the operator."""

        return self._linear_velocity_cap

    def set_speed_mps(self, speed_mps: float) -> None:
        """Set the forward-speed cap; the value comes from an operator request."""

        low, high = self.config.slow_linear_velocity_mps, self.config.fast_linear_velocity_mps
        if not math.isfinite(speed_mps):
            raise ValueError("speed must be finite")
        if not low <= speed_mps <= high:
            raise ValueError(f"speed must be between {low:.2f} and {high:.2f} m/s")
        self._linear_velocity_cap = float(speed_mps)

    @staticmethod
    def _samples(low: float, high: float, count: int) -> tuple[float, ...]:
        values = {float(value) for value in np.linspace(low, high, count)}
        # Stop and rotate must remain available even outside acceleration bounds.
        values.add(0.0)
        return tuple(sorted(values))

    def _window(
        self, velocity: Velocity, speed_cap: float | None = None
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        config = self.config
        dt = 1.0 / config.control_hz
        cap = self.speed_mps if speed_cap is None else min(self.speed_mps, max(0.0, speed_cap))
        linear_high = min(cap, velocity.linear + config.linear_acceleration_mps2 * dt)
        # If the operator lowers the cap while the robot is above it, allow a
        # bounded braking command instead of sampling an invalid interval.
        linear_low = min(
            linear_high,
            max(config.min_linear_velocity_mps, velocity.linear - config.linear_acceleration_mps2 * dt),
        )
        angular_low = max(
            -config.max_angular_velocity_rps, velocity.angular - config.angular_acceleration_rps2 * dt
        )
        angular_high = min(
            config.max_angular_velocity_rps, velocity.angular + config.angular_acceleration_rps2 * dt
        )
        return (
            self._samples(linear_low, linear_high, config.linear_samples),
            self._samples(angular_low, angular_high, config.angular_samples),
        )

    def _goal_speed_limit(self, distance_to_goal: float) -> float:
        """Return a speed cap that can stop before the exact goal."""

        remaining = max(0.0, distance_to_goal - self.config.goal_braking_buffer_m)
        return min(
            self.speed_mps,
            math.sqrt(2.0 * self.config.goal_braking_deceleration_mps2 * remaining),
        )

    def _rollout(self, linear: float, angular: float) -> tuple[Pose2D, ...]:
        """Integrate one constant-velocity arc in the robot frame."""

        dt = self.config.prediction_step_seconds
        steps = math.ceil(self.config.prediction_horizon_seconds / dt)
        pose = Pose2D(0.0, 0.0, 0.0)
        arc = [pose]
        for _ in range(steps):
            pose = Pose2D(
                pose.x + linear * math.cos(pose.yaw) * dt,
                pose.y + linear * math.sin(pose.yaw) * dt,
                normalize_angle(pose.yaw + angular * dt),
            )
            arc.append(pose)
        return tuple(arc)

    def _is_trajectory_safe(self, trajectory: Sequence[Pose2D]) -> bool:
        start = trajectory[0].to_point()
        if not self.cost_map.is_raw_free(start):
            return False
        # From a free start the inflated costmap stays a hard constraint.  From
        # inside inflation an arc is admissible while it stays outside raw
        # geometry and never moves closer to it, so the robot can drive out.
        if self.cost_map.is_free(start):
            return all(self.cost_map.is_free(item.to_point()) for item in trajectory)
        clearance = self.cost_map.clearance_at(start)
        return all(
            self.cost_map.is_raw_free(item.to_point())
            and self.cost_map.clearance_at(item.to_point()) + 1e-9 >= clearance
            for item in trajectory
        )

    def _progress(
        self, field: ProgressField, start: Point2D, end: Point2D, target: Point2D
    ) -> float:
        """Normalised headway, from the cost-to-go field or the local target.

        The field is authoritative where it is defined; closing distance to the
        local target covers the arcs that leave the field's reachable set.
        """

        change_m = field.progress_m(start, end)
        if change_m == math.inf:
            return 1.0
        lookahead = self.config.lookahead_distance_m
        by_field = change_m / lookahead if math.isfinite(change_m) else -1.0
        by_target = (distance(start, target) - distance(end, target)) / lookahead
        return max(-1.0, min(1.0, max(by_field, by_target)))

    def _select(self, candidates: list[_Candidate], start_is_free: bool) -> _Candidate:
        """Choose the arc to drive from the ones that passed collision checks.

        Reverse is an escape manoeuvre.  Once the robot is back in free map
        space, prefer an arc that makes headway, then one that turns to look
        for headway, then any non-reversing arc; only fall through to the
        outright best when none of those survived, so the robot re-accelerates
        instead of backing along the whole global path.
        """

        best = max(candidates, key=lambda item: item.rank)
        if not start_is_free:
            return best
        # Anything below half a cycle's worth of angular acceleration is drift,
        # not a deliberate turn.
        min_turn_rate = 0.5 * self.config.angular_acceleration_rps2 / self.config.control_hz
        preferences = (
            lambda item: item.linear > 0.0 and item.progress > 1e-4,
            lambda item: item.linear >= 0.0 and abs(item.angular) >= min_turn_rate,
            lambda item: item.linear >= 0.0,
        )
        for accept in preferences:
            allowed = [item for item in candidates if accept(item)]
            if allowed:
                return max(allowed, key=lambda item: item.rank)
        return best

    def _terminal_safe(
        self,
        pose: Pose2D,
        command: Velocity,
        obstacles: Sequence[Point2D],
    ) -> tuple[bool, tuple[Pose2D, ...], float]:
        """Collision-check a terminal command using map and live scan geometry."""

        local = self._rollout(command.linear, command.angular)
        world = tuple(_to_world(pose, step) for step in local)
        if not self._is_trajectory_safe(world):
            return False, world, 0.0
        centre_clearance = min_distance(_ORIGIN, obstacles)
        if obstacles:
            required = min(
                self.config.robot_radius_m + self.config.local_safety_margin_m,
                centre_clearance,
            )
            if any(
                min_distance(step.to_point(), obstacles) + 1e-9 < required
                for step in local
            ):
                return False, world, centre_clearance
        return True, world, centre_clearance

    def _terminal_command(
        self,
        *,
        pose: Pose2D,
        goal: Point2D,
        phase: GoalPhase,
        goal_yaw: float | None,
    ) -> Velocity:
        """Compute a bounded final pose command without path-mode oscillation."""

        config = self.config
        if phase is GoalPhase.VERIFYING:
            return STOP
        if phase is GoalPhase.SETTLING_YAW:
            if goal_yaw is not None:
                yaw_error = normalize_angle(goal_yaw - pose.yaw)
                error = abs(yaw_error)
                if error > config.goal_yaw_tolerance_rad:
                    remaining = max(0.0, error - config.goal_yaw_tolerance_rad)
                    angular = min(
                        config.goal_yaw_max_angular_velocity_rps,
                        config.terminal_yaw_gain * error,
                        math.sqrt(2.0 * config.terminal_yaw_deceleration_rps2 * remaining),
                    )
                    if error > config.goal_yaw_slowdown_tolerance_rad:
                        angular = max(config.goal_yaw_min_angular_velocity_rps, angular)
                    return Velocity(0.0, math.copysign(angular, yaw_error))

        position = pose.to_point()
        distance_to_goal = distance(position, goal)
        if distance_to_goal <= config.goal_tolerance_m:
            return STOP
        target_heading = math.atan2(goal.y - pose.y, goal.x - pose.x)
        direction = 1.0
        heading_error = normalize_angle(target_heading - pose.yaw)
        if abs(heading_error) > math.pi / 2.0:
            direction = -1.0
            heading_error = normalize_angle(target_heading + math.pi - pose.yaw)
        angular = max(
            -config.terminal_angular_velocity_rps,
            min(config.terminal_angular_velocity_rps, config.terminal_heading_gain * heading_error),
        )
        if abs(heading_error) > config.terminal_heading_rotate_threshold_rad:
            return Velocity(0.0, angular)
        stopping_speed = math.sqrt(
            2.0
            * config.goal_braking_deceleration_mps2
            * max(0.0, distance_to_goal - config.goal_tolerance_m)
        )
        linear = min(config.terminal_linear_velocity_mps, stopping_speed)
        return Velocity(direction * linear, angular)

    def command(
        self,
        *,
        pose: Pose2D,
        velocity: Velocity,
        path_world: Sequence[Point2D],
        obstacles_robot: Sequence[Point2D],
        previous: Velocity,
        progress_field: ProgressField,
        goal_yaw: float | None = None,
        phase: GoalPhase | None = None,
        goal: Point2D | None = None,
    ) -> ControlDecision:
        if not path_world:
            raise ValueError("global path is empty")
        config = self.config
        position = pose.to_point()
        # Static geometry is already represented by the map/costmap.  Feeding
        # its own returns back as obstacles can make every forward arc look
        # unsafe next to an inflated pillar and leave the robot rotating
        # forever.  Returns the static map cannot explain stay active for
        # dynamic-obstacle avoidance.
        obstacles = tuple(
            point
            for point in obstacles_robot
            if self.cost_map.static_is_free(transform_point(pose, point))
        )
        goal_point = goal or path_world[-1]
        goal_distance = distance(position, goal_point)
        if phase is None and goal_yaw is not None and goal_distance <= config.goal_terminal_entry_m:
            # Compatibility for direct LocalPlanner callers; CustomNavigator supplies
            # an explicit phase for every active mission.
            phase = (
                GoalPhase.SETTLING_YAW
                if abs(normalize_angle(goal_yaw - pose.yaw)) > config.goal_yaw_tolerance_rad
                else GoalPhase.SETTLING_POSITION
            )
        if phase in {
            GoalPhase.SETTLING_POSITION,
            GoalPhase.SETTLING_YAW,
            GoalPhase.VERIFYING,
        }:
            command = self._terminal_command(
                pose=pose,
                goal=goal_point,
                phase=phase,
                goal_yaw=goal_yaw,
            )
            safe, trajectory, centre_clearance = self._terminal_safe(
                pose, command, obstacles
            )
            clearance = min(self.cost_map.clearance_at(position), centre_clearance)
            return ControlDecision(
                command if safe else STOP,
                trajectory if safe else (pose,),
                _reported_clearance(clearance, config.robot_radius_m),
                safe,
                DecisionReason.SELECTED
                if safe
                else DecisionReason.NO_SAFE_LIDAR_TRAJECTORY,
            )
        start_is_free = self.cost_map.is_free(position)
        target = target_point(position, path_world, config.lookahead_distance_m)
        speed_cap = self._goal_speed_limit(goal_distance)
        score_speed_cap = max(self.speed_mps, 1e-6)
        centre_clearance = min_distance(_ORIGIN, obstacles)
        # Normally an arc must clear the footprint.  If the robot already sits
        # closer than that, it only has to avoid making the gap smaller.
        admissible = min(config.robot_radius_m + config.local_safety_margin_m, centre_clearance)

        def score(linear: float, angular: float, end: Pose2D, end_local: Point2D, progress: float) -> float:
            # Reversing arcs are judged on where the robot is backing towards.
            travel_heading = end.yaw if linear >= 0.0 else normalize_angle(end.yaw + math.pi)
            target_heading = math.atan2(target.y - end.y, target.x - end.x)
            heading = (math.cos(normalize_angle(target_heading - travel_heading)) + 1.0) * 0.5
            clearance = min(min_distance(end_local, obstacles), self.cost_map.clearance_at(end.to_point()))
            deviation = _distance_to_path(end.to_point(), path_world)
            smoothness = 0.5 * (
                abs(linear - previous.linear) / (score_speed_cap - config.min_linear_velocity_mps)
                + abs(angular - previous.angular) / (2.0 * config.max_angular_velocity_rps)
            )
            stationary = abs(linear) < 1e-9 and abs(angular) < 1e-9
            return (
                config.score_progress * progress
                + config.score_heading * heading
                + config.score_clearance * min(1.0, clearance / config.preferred_clearance_m)
                + config.score_speed
                * (1.0 - min(1.0, abs(abs(linear) - speed_cap) / score_speed_cap))
                - config.score_path_deviation * min(1.0, deviation / config.lookahead_distance_m)
                - config.score_smoothness * smoothness
                - (config.reverse_penalty if linear < 0.0 else 0.0)
                - (config.score_stationary if stationary else 0.0)
            )

        linear_samples, angular_samples = self._window(velocity, speed_cap)
        candidates: list[_Candidate] = []
        lidar_blocked = False
        for linear in linear_samples:
            for angular in angular_samples:
                local = self._rollout(linear, angular)
                world = tuple(_to_world(pose, step) for step in local)
                if not self._is_trajectory_safe(world):
                    continue
                clearance = min(min_distance(step.to_point(), obstacles) for step in local)
                if clearance + 1e-9 < admissible:
                    lidar_blocked = True
                    continue
                progress = self._progress(progress_field, position, world[-1].to_point(), target)
                end_local = local[-1].to_point()
                candidates.append(
                    _Candidate(
                        score(linear, angular, world[-1], end_local, progress),
                        linear,
                        angular,
                        world,
                        clearance,
                        progress,
                    )
                )

        if not candidates:
            reason = (
                DecisionReason.NO_SAFE_LIDAR_TRAJECTORY
                if lidar_blocked
                else DecisionReason.NO_SAFE_MAP_TRAJECTORY
            )
            return ControlDecision(
                STOP, (pose,), _reported_clearance(centre_clearance, config.robot_radius_m), False, reason
            )

        chosen = self._select(candidates, start_is_free)
        return ControlDecision(
            Velocity(chosen.linear, chosen.angular),
            chosen.trajectory,
            _reported_clearance(chosen.clearance, config.robot_radius_m),
            True,
            DecisionReason.SELECTED if start_is_free else DecisionReason.ESCAPING,
        )
