"""ROS-facing configuration for the custom navigation backend."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class NavigationConfig:
    """Planner/controller parameters tuned for the current AMR baseline."""

    robot_radius_m: float = 0.22
    safety_margin_m: float = 0.05
    local_safety_margin_m: float = 0.03
    goal_tolerance_m: float = 0.12
    goal_terminal_entry_m: float = 0.20
    goal_terminal_exit_m: float = 0.14
    goal_yaw_tolerance_rad: float = 0.20
    goal_yaw_min_angular_velocity_rps: float = 0.04
    goal_yaw_slowdown_tolerance_rad: float = 0.35
    goal_yaw_max_angular_velocity_rps: float = 0.55
    goal_braking_deceleration_mps2: float = 0.80
    goal_braking_buffer_m: float = 0.08
    terminal_linear_velocity_mps: float = 0.08
    terminal_angular_velocity_rps: float = 0.35
    terminal_heading_rotate_threshold_rad: float = 0.40
    terminal_heading_gain: float = 1.20
    terminal_yaw_gain: float = 0.90
    terminal_yaw_deceleration_rps2: float = 0.75
    stable_linear_velocity_mps: float = 0.02
    stable_angular_velocity_rps: float = 0.03
    stable_goal_cycles: int = 5
    waypoint_spacing_m: float = 0.25
    preferred_clearance_m: float = 0.38
    clearance_cost_weight: float = 2.0
    lookahead_distance_m: float = 0.45
    min_linear_velocity_mps: float = -0.08
    slow_linear_velocity_mps: float = 0.06
    default_linear_velocity_mps: float = 0.16
    fast_linear_velocity_mps: float = 0.22
    max_linear_velocity_mps: float = 0.22
    max_angular_velocity_rps: float = 0.55
    linear_acceleration_mps2: float = 0.50
    angular_acceleration_rps2: float = 1.20
    control_hz: float = 5.0
    heartbeat_hz: float = 20.0
    prediction_horizon_seconds: float = 1.5
    prediction_step_seconds: float = 0.10
    linear_samples: int = 7
    angular_samples: int = 15
    sensor_timeout_seconds: float = 0.60
    no_progress_timeout_seconds: float = 12.0
    minimum_progress_m: float = 0.04
    active_turn_rps: float = 0.05
    close_obstacle_clearance_m: float = 0.30
    temporary_obstacle_ttl_seconds: float = 2.0
    score_progress: float = 2.4
    score_heading: float = 1.5
    score_clearance: float = 2.8
    score_speed: float = 0.6
    score_path_deviation: float = 1.8
    score_smoothness: float = 0.25
    score_stationary: float = 0.35
    reverse_penalty: float = 0.35
    lidar_count: int = 0
    lidar_sensor_x_m: float = 0.0
    lidar_sensor_y_m: float = 0.0
    lidar_sensor_yaw_rad: float = 0.0


DEFAULT_CONFIG = NavigationConfig()
