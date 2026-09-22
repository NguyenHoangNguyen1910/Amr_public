"""Dynamic-window command selection and final-pose control."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from .config import NavigationConfig
from .global_planner import CostMap
from .types import (
    STOP,
    ControlDecision,
    Point2D,
    Pose2D,
    Velocity,
    distance,
    min_distance,
    normalize_angle,
    target_point,
    transform_point,
)

_ORIGIN = Point2D(0.0, 0.0)


def _to_world(pose: Pose2D, local: Pose2D) -> Pose2D:
    point = transform_point(pose, local.to_point())
    return Pose2D(point.x, point.y, normalize_angle(pose.yaw + local.yaw))


@dataclass(frozen=True)
class _Candidate:
    score: float
    command: Velocity
    clearance: float

    @property
    def rank(self) -> tuple[float, float, float, float]:
        return self.score, self.clearance, self.command.linear, -abs(self.command.angular)


class LocalPlanner:
    def __init__(self, cost_map: CostMap, config: NavigationConfig):
        self.cost_map = cost_map
        self.config = config

    @staticmethod
    def _samples(low: float, high: float, count: int) -> tuple[float, ...]:
        return tuple(sorted({0.0, *(float(item) for item in np.linspace(low, high, count))}))

    def _window(self, velocity: Velocity) -> tuple[tuple[float, ...], tuple[float, ...]]:
        config = self.config
        dt = 1.0 / config.control_hz
        linear_high = min(
            config.default_linear_velocity_mps,
            velocity.linear + config.linear_acceleration_mps2 * dt,
        )
        linear_low = min(
            linear_high,
            max(
                config.min_linear_velocity_mps,
                velocity.linear - config.linear_acceleration_mps2 * dt,
            ),
        )
        angular_low = max(
            -config.max_angular_velocity_rps,
            velocity.angular - config.angular_acceleration_rps2 * dt,
        )
        angular_high = min(
            config.max_angular_velocity_rps,
            velocity.angular + config.angular_acceleration_rps2 * dt,
        )
        return (
            self._samples(linear_low, linear_high, config.linear_samples),
            self._samples(angular_low, angular_high, config.angular_samples),
        )

    def _rollout(self, command: Velocity) -> tuple[Pose2D, ...]:
        config = self.config
        dt = config.prediction_step_seconds
        pose = Pose2D(0.0, 0.0, 0.0)
        trajectory = [pose]
        for _ in range(math.ceil(config.prediction_horizon_seconds / dt)):
            pose = Pose2D(
                pose.x + command.linear * math.cos(pose.yaw) * dt,
                pose.y + command.linear * math.sin(pose.yaw) * dt,
                normalize_angle(pose.yaw + command.angular * dt),
            )
            trajectory.append(pose)
        return tuple(trajectory)

    def _map_safe(self, trajectory: Sequence[Pose2D]) -> bool:
        start = trajectory[0].to_point()
        if not self.cost_map.is_raw_free(start):
            return False
        if self.cost_map.is_free(start):
            return all(self.cost_map.is_free(item.to_point()) for item in trajectory)
        clearance = self.cost_map.clearance_at(start)
        return all(
            self.cost_map.is_raw_free(item.to_point())
            and self.cost_map.clearance_at(item.to_point()) + 1e-9 >= clearance
            for item in trajectory
        )

    def _trajectories(
        self, pose: Pose2D, command: Velocity
    ) -> tuple[tuple[Pose2D, ...], tuple[Pose2D, ...]]:
        local = self._rollout(command)
        return local, tuple(_to_world(pose, item) for item in local)

    def _terminal_command(self, pose: Pose2D, goal: Point2D) -> Velocity:
        config = self.config
        distance_to_goal = distance(pose.to_point(), goal)
        if distance_to_goal > config.goal_tolerance_m:
            target_yaw = math.atan2(goal.y - pose.y, goal.x - pose.x)
            error = normalize_angle(target_yaw - pose.yaw)
            angular = max(
                -config.terminal_angular_velocity_rps,
                min(config.terminal_angular_velocity_rps, config.terminal_heading_gain * error),
            )
            if abs(error) > config.terminal_heading_rotate_threshold_rad:
                return Velocity(0.0, angular)
            return Velocity(config.terminal_linear_velocity_mps, angular)
        if goal.yaw is None:
            return STOP
        error = normalize_angle(goal.yaw - pose.yaw)
        if abs(error) <= config.goal_yaw_tolerance_rad:
            return STOP
        angular = max(
            -config.terminal_angular_velocity_rps,
            min(config.terminal_angular_velocity_rps, config.terminal_yaw_gain * error),
        )
        return Velocity(0.0, angular)

    def _obstacles(self, pose: Pose2D, points: Sequence[Point2D]) -> tuple[Point2D, ...]:
        return tuple(
            point
            for point in points
            if self.cost_map.static_is_free(transform_point(pose, point))
        )

    def _safe_command(
        self, pose: Pose2D, command: Velocity, obstacles: Sequence[Point2D]
    ) -> tuple[bool, float]:
        local, world = self._trajectories(pose, command)
        if not self._map_safe(world):
            return False, 0.0
        clearance = min(min_distance(item.to_point(), obstacles) for item in local)
        required = min(
            self.config.robot_radius_m + self.config.local_safety_margin_m,
            min_distance(_ORIGIN, obstacles),
        )
        return clearance + 1e-9 >= required, clearance

    def _score(
        self,
        command: Velocity,
        end: Pose2D,
        target: Point2D,
        progress: float,
        clearance: float,
    ) -> float:
        travel_yaw = end.yaw if command.linear >= 0.0 else normalize_angle(end.yaw + math.pi)
        target_yaw = math.atan2(target.y - end.y, target.x - end.x)
        heading = (math.cos(normalize_angle(target_yaw - travel_yaw)) + 1.0) * 0.5
        config = self.config
        speed = abs(command.linear) / max(config.default_linear_velocity_mps, 1e-6)
        map_clearance = self.cost_map.clearance_at(end.to_point())
        clearance = min(clearance, map_clearance)
        return (
            config.score_progress * progress
            + config.score_heading * heading
            + config.score_clearance * min(1.0, clearance / config.lookahead_distance_m)
            + config.score_speed * min(1.0, speed)
            - (config.reverse_penalty if command.linear < 0.0 else 0.0)
        )

    def command(
        self,
        pose: Pose2D,
        velocity: Velocity,
        path: Sequence[Point2D],
        obstacles_robot: Sequence[Point2D],
        progress_field,
        goal: Point2D,
        final: bool,
    ) -> ControlDecision:
        obstacles = self._obstacles(pose, obstacles_robot)
        if final:
            command = self._terminal_command(pose, goal)
            safe, clearance = self._safe_command(pose, command, obstacles)
            return ControlDecision(
                command if safe else STOP,
                clearance,
                safe,
                not self.cost_map.is_free(pose.to_point()),
            )

        position = pose.to_point()
        target = target_point(position, path, self.config.lookahead_distance_m)
        candidates: list[_Candidate] = []
        linear_samples, angular_samples = self._window(velocity)
        for linear in linear_samples:
            for angular in angular_samples:
                command = Velocity(linear, angular)
                safe, clearance = self._safe_command(pose, command, obstacles)
                if not safe:
                    continue
                _, world = self._trajectories(pose, command)
                progress = progress_field.progress_m(position, world[-1].to_point())
                candidates.append(
                    _Candidate(
                        self._score(command, world[-1], target, progress, clearance),
                        command,
                        clearance,
                    )
                )
        if not candidates:
            return ControlDecision(STOP, None, False, False)
        choice = max(candidates, key=lambda item: item.rank)
        return ControlDecision(
            choice.command,
            max(0.0, choice.clearance - self.config.robot_radius_m),
            True,
            not self.cost_map.is_free(position),
        )
