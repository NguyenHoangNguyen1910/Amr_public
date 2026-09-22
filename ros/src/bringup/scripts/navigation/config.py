"""Navigation parameters with conservative robot-safe defaults."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NavigationConfig:
    robot_radius_m: float = 0.05
    safety_margin_m: float = 0.05
    local_safety_margin_m: float = 0.03
    goal_tolerance_m: float = 0.12
    goal_terminal_entry_m: float = 0.20
    goal_terminal_exit_m: float = 0.14
    goal_yaw_tolerance_rad: float = 0.20
    stable_linear_velocity_mps: float = 0.02
    stable_angular_velocity_rps: float = 0.03
    stable_goal_cycles: int = 5
    terminal_linear_velocity_mps: float = 0.08
    terminal_angular_velocity_rps: float = 0.35
    terminal_heading_rotate_threshold_rad: float = 0.40
    terminal_heading_gain: float = 1.20
    terminal_yaw_gain: float = 0.90
    waypoint_spacing_m: float = 0.25
    lookahead_distance_m: float = 0.45
    min_linear_velocity_mps: float = -0.22
    default_linear_velocity_mps: float = 0.30
    max_linear_velocity_mps: float = 0.30
    max_angular_velocity_rps: float = 0.55
    linear_acceleration_mps2: float = 0.50
    angular_acceleration_rps2: float = 1.20
    control_hz: float = 5.0
    prediction_horizon_seconds: float = 1.5
    prediction_step_seconds: float = 0.10
    linear_samples: int = 7
    angular_samples: int = 15
    sensor_timeout_seconds: float = 0.60
    no_progress_timeout_seconds: float = 12.0
    minimum_progress_m: float = 0.04
    temporary_obstacle_ttl_seconds: float = 2.0
    score_progress: float = 2.4
    score_heading: float = 1.5
    score_clearance: float = 2.8
    score_speed: float = 0.6
    reverse_penalty: float = 0.35

    def __post_init__(self) -> None:
        positive = (
            self.robot_radius_m,
            self.safety_margin_m,
            self.goal_tolerance_m,
            self.goal_terminal_exit_m,
            self.goal_terminal_entry_m,
            self.waypoint_spacing_m,
            self.lookahead_distance_m,
            self.default_linear_velocity_mps,
            self.max_linear_velocity_mps,
            self.control_hz,
            self.prediction_horizon_seconds,
            self.prediction_step_seconds,
            self.sensor_timeout_seconds,
            self.no_progress_timeout_seconds,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("navigation configuration values must be positive")
        if self.goal_terminal_exit_m > self.goal_terminal_entry_m:
            raise ValueError("terminal exit distance must not exceed entry distance")
        if self.linear_samples < 2 or self.angular_samples < 2:
            raise ValueError("dynamic window needs at least two samples per axis")


DEFAULT_CONFIG = NavigationConfig()
