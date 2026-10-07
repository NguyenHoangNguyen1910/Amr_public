import math
import unittest
from dataclasses import replace
from unittest.mock import patch

import numpy as np

from navigation.config import NavigationConfig
from navigation.global_planner import OccupancyGrid, PlanningError, GlobalPlanner, CostMap
from navigation.local_planner import LocalPlanner
from navigation.navigator import Navigator
from navigation.types import GoalPhase, NavigationInput, Point2D, Pose2D, State, STOP, Velocity


class NavigationCoreTest(unittest.TestCase):
    def setUp(self):
        self.config = NavigationConfig(stable_goal_cycles=1)
        raw = np.zeros((60, 60), dtype=np.bool_)
        self.grid = OccupancyGrid(
            raw,
            min_x=-3.0,
            min_y=-3.0,
            resolution_m=0.1,
            inflation_radius_m=self.config.robot_radius_m + self.config.safety_margin_m,
        )

    @staticmethod
    def navigation_input(x=0.0, y=0.0, yaw=0.0):
        return NavigationInput(Pose2D(x, y, yaw), STOP, ())

    def test_session_tracks_a_clear_goal(self):
        navigator = Navigator(self.grid, self.config)
        navigator.plan(Point2D(1.0, 0.0), self.navigation_input(), now=0.0)

        result = navigator.step(self.navigation_input(), now=0.1)

        self.assertEqual(result.state, State.NAVIGATING)
        self.assertGreater(result.command.linear, 0.0)
        self.assertEqual(result.path[-1], Point2D(1.0, 0.0))

    def test_no_progress_watchdog_uses_observed_motion_not_command(self):
        config = replace(self.config, no_progress_timeout_seconds=1.0)
        navigator = Navigator(self.grid, config)
        navigation_input = self.navigation_input()
        navigator.plan(Point2D(1.0, 0.0), navigation_input, now=0.0)

        first = navigator.step(navigation_input, now=0.1)
        self.assertNotEqual(first.command, STOP)
        result = navigator.step(navigation_input, now=1.2)

        self.assertEqual(result.state, State.BLOCKED)
        self.assertEqual(result.command, STOP)

    def test_final_yaw_progress_resets_watchdog_but_spin_without_progress_does_not(self):
        config = replace(self.config, no_progress_timeout_seconds=1.0)
        navigator = Navigator(self.grid, config)
        goal = Point2D(0.0, 0.0, math.pi / 2.0)
        initial = self.navigation_input()
        navigator.plan(goal, initial, now=0.0)

        first = navigator.step(initial, now=0.1)
        self.assertGreater(first.command.angular, 0.0)
        turned = self.navigation_input(yaw=0.1)
        progressing = navigator.step(turned, now=1.2)
        self.assertEqual(progressing.state, State.NAVIGATING)

        stalled = navigator.step(turned, now=2.3)
        self.assertEqual(stalled.state, State.BLOCKED)
        self.assertEqual(stalled.command, STOP)

    def test_terminal_approach_tracks_bearing_rotation_not_final_goal_yaw(self):
        config = replace(self.config, no_progress_timeout_seconds=1.0)
        navigator = Navigator(self.grid, config)
        goal = Point2D(0.0, 0.0, 0.0)
        initial = self.navigation_input(x=0.13, yaw=0.5)
        navigator.plan(goal, initial, now=0.0)

        first = navigator.step(initial, now=0.1)
        self.assertGreater(first.command.angular, 0.0)
        rotated = self.navigation_input(x=0.13, yaw=0.6)
        progressing = navigator.step(rotated, now=1.2)
        self.assertEqual(progressing.state, State.NAVIGATING)

        stalled = navigator.step(rotated, now=2.3)
        self.assertEqual(stalled.state, State.BLOCKED)
        self.assertEqual(stalled.command, STOP)

    def test_local_velocity_samples_collapse_floating_point_zero(self):
        samples = LocalPlanner._samples(
            -0.10000000000000003,
            0.09999999999999998,
            7,
        )

        near_zero = tuple(value for value in samples if abs(value) <= 1e-9)
        self.assertEqual(near_zero, (0.0,))

    def test_final_yaw_does_not_switch_to_bearing_on_one_position_sample(self):
        config = replace(self.config, goal_tolerance_m=0.06, stable_goal_cycles=8)
        navigator = Navigator(self.grid, config)
        goal = Point2D(0.0, 0.0, math.pi / 2)
        initial = self.navigation_input(x=0.059, yaw=2.0)
        navigator.plan(goal, initial, now=0.0)
        first = navigator.step(initial, now=0.1)
        self.assertLess(first.command.angular, 0.0)
        # A 2 mm pose fluctuation must not replace final yaw with atan2(goal-pose).
        result = navigator.step(self.navigation_input(x=0.061, yaw=1.9), now=0.3)
        self.assertLess(result.command.angular, 0.0)
        self.assertEqual(result.command.linear, 0.0)
        self.assertEqual(navigator._terminal_yaw_error(result.pose)[0], 'goal')

    def test_final_yaw_latch_does_not_relax_success_position_tolerance(self):
        config = replace(self.config, goal_tolerance_m=0.06, stable_goal_cycles=3)
        navigator = Navigator(self.grid, config)
        initial = self.navigation_input(x=0.059, yaw=-0.5)
        navigator.plan(Point2D(0.0, 0.0, -math.pi / 2), initial, now=0.0)
        navigator.step(initial, now=0.1)
        outside = self.navigation_input(x=0.061, yaw=-math.pi / 2)
        for t in (0.3, 0.5):
            result = navigator.step(outside, now=t)
            self.assertEqual(result.state, State.NAVIGATING)
            self.assertEqual(result.command, STOP)
        # Persistent position error must still trigger position recovery.
        result = navigator.step(outside, now=0.7)
        self.assertNotEqual(result.command, STOP)
        self.assertEqual(navigator._terminal_yaw_error(outside.pose)[0], 'bearing')

    def test_planning_failure_breaks_consecutive_stability(self):
        config = replace(self.config, stable_goal_cycles=3)
        navigator = Navigator(self.grid, config)
        pose = self.navigation_input()
        navigator.plan(Point2D(0.0, 0.0, 0.0), pose, now=0.0)
        navigator.step(pose, now=0.1)
        navigator.step(pose, now=0.2)
        from navigation.global_planner import PlanningErrorCode
        with patch.object(navigator.planner, 'refresh', side_effect=PlanningError(
                PlanningErrorCode.GOAL_BLOCKED, 'test obstruction')):
            navigator.step(pose, now=0.3)
        self.assertEqual(navigator.stable_cycles, 0)
        self.assertIn('GOAL_BLOCKED', navigator.blocked_reason)
        self.assertEqual(navigator.step(pose, now=0.4).state, State.NAVIGATING)

    def test_new_goal_resets_terminal_yaw_latch(self):
        navigator = Navigator(self.grid, self.config)
        pose = self.navigation_input(yaw=-0.5)
        navigator.plan(Point2D(0.0, 0.0, -math.pi / 2), pose, now=0.0)
        navigator.step(pose, now=0.1)
        navigator.plan(Point2D(0.15, 0.0, -math.pi / 2), pose, now=0.2)
        result = navigator.step(pose, now=0.3)
        self.assertGreater(result.command.angular, 0.0)

    def test_final_yaw_latch_releases_immediately_after_large_displacement(self):
        navigator = Navigator(self.grid, self.config)
        initial = self.navigation_input()
        navigator.plan(Point2D(0.0, 0.0, math.pi / 2), initial, now=0.0)
        navigator.step(initial, now=0.1)
        self.assertTrue(navigator.local_planner.aligning_goal_yaw)
        navigator.step(self.navigation_input(x=0.25), now=0.3)
        self.assertFalse(navigator.local_planner.aligning_goal_yaw)

    def test_final_yaw_still_requires_safe_command(self):
        navigator = Navigator(self.grid, self.config)
        initial = self.navigation_input()
        navigator.plan(Point2D(0.0, 0.0, math.pi / 2), initial, now=0.0)
        with patch.object(navigator.local_planner, '_safe_command', return_value=(False, 0.0)):
            result = navigator.step(initial, now=0.1)
        self.assertEqual(result.command, STOP)
        self.assertEqual(result.state, State.NAVIGATING)
        self.assertIn('UNSAFE_COMMAND', navigator.blocked_reason)

    def test_final_yaw_wraps_across_pi_and_completes_only_when_stable(self):
        config = replace(self.config, stable_goal_cycles=3)
        navigator = Navigator(self.grid, config)
        initial = self.navigation_input(yaw=math.pi - 0.3)
        navigator.plan(Point2D(0.0, 0.0, -math.pi + 0.3), initial, now=0.0)
        self.assertGreater(navigator.step(initial, now=0.1).command.angular, 0.0)
        final = self.navigation_input(yaw=-math.pi + 0.3)
        self.assertEqual(navigator.step(final, now=0.3).state, State.NAVIGATING)
        self.assertEqual(navigator.step(final, now=0.5).state, State.NAVIGATING)
        self.assertEqual(navigator.step(final, now=0.7).state, State.SUCCEEDED)

    def test_session_finishes_a_stable_goal(self):
        navigator = Navigator(self.grid, self.config)
        goal = Point2D(0.0, 0.0, math.pi / 2.0)
        navigator.plan(goal, self.navigation_input(yaw=math.pi / 2.0), now=0.0)

        result = navigator.step(self.navigation_input(yaw=math.pi / 2.0), now=0.1)

        self.assertEqual(result.state, State.SUCCEEDED)
        self.assertEqual(result.command, STOP)

    def test_cancel_stops_the_session(self):
        navigator = Navigator(self.grid, self.config)
        navigator.plan(Point2D(1.0, 0.0), self.navigation_input(), now=0.0)
        navigator.cancel()

        result = navigator.step(self.navigation_input(), now=0.1)

        self.assertEqual(result.state, State.CANCELLED)
        self.assertEqual(result.command, STOP)

    def test_blocked_goal_is_rejected(self):
        raw = np.zeros((10, 10), dtype=np.bool_)
        raw[5, 5] = True
        grid = OccupancyGrid(
            raw,
            min_x=0.0,
            min_y=0.0,
            resolution_m=0.1,
            inflation_radius_m=0.0,
        )
        with self.assertRaises(PlanningError):
            GlobalPlanner(CostMap(grid, 1.0), Point2D(0.55, 0.55), self.config)

    def test_terminal_hysteresis_does_not_flip_at_fixed_distance(self):
        config = replace(self.config, goal_terminal_entry_m=0.30, goal_terminal_exit_m=0.40)
        navigator = Navigator(self.grid, config)
        pose = self.navigation_input(x=0.20)
        navigator.plan(Point2D(0.0, 0.0, 0.0), pose, now=0.0)
        for i in range(8):
            navigator.step(pose, now=(i + 1) * 0.2)
            self.assertEqual(navigator.phase, GoalPhase.FINAL)
        navigator.step(self.navigation_input(x=0.35), now=1.8)
        self.assertEqual(navigator.phase, GoalPhase.FINAL)
        navigator.step(self.navigation_input(x=0.41), now=2.0)
        self.assertEqual(navigator.phase, GoalPhase.TRACKING)

    def test_invalid_terminal_hysteresis_is_rejected(self):
        for exit_distance in (0.08, 0.30):
            with self.assertRaises(ValueError):
                replace(self.config, goal_terminal_entry_m=0.30, goal_terminal_exit_m=exit_distance)

    def test_tracking_rotation_is_progress_but_oscillation_is_not(self):
        config = replace(self.config, no_progress_timeout_seconds=1.0)
        navigator = Navigator(self.grid, config)
        initial = self.navigation_input(yaw=math.pi)
        navigator.plan(Point2D(1.0, 0.0), initial, now=0.0)
        navigator.step(initial, now=0.1)
        for i in range(1, 5):
            result = navigator.step(self.navigation_input(yaw=math.pi - i * 0.1), now=i * 0.8)
            self.assertEqual(result.state, State.NAVIGATING)
        # Revisiting already observed headings is not new progress.
        navigator.step(self.navigation_input(yaw=math.pi - 0.3), now=3.6)
        result = navigator.step(self.navigation_input(yaw=math.pi - 0.4), now=4.3)
        self.assertEqual(result.state, State.BLOCKED)
        self.assertEqual(result.command, STOP)

    def test_map_update_replans_and_does_not_reset_watchdog(self):
        navigator = Navigator(self.grid, self.config)
        pose = self.navigation_input()
        navigator.plan(Point2D(1.0, 0.0), pose, now=0.0)
        original_path = navigator.path
        raw = self.grid.raw.copy()
        raw[29:32, 35] = True
        changed = OccupancyGrid(raw, min_x=-3.0, min_y=-3.0, resolution_m=0.1,
                                inflation_radius_m=0.1)
        navigator.update_map(changed, now=1.0)
        result = navigator.step(pose, now=1.0)
        self.assertEqual(result.state, State.NAVIGATING)
        self.assertNotEqual(result.path, original_path)
        self.assertEqual(navigator.last_progress, 0.0)
        result = navigator.step(pose, now=13.0)
        self.assertEqual(result.state, State.BLOCKED)

    def test_updated_map_blocking_goal_stops_session(self):
        navigator = Navigator(self.grid, self.config)
        navigator.plan(Point2D(1.05, 0.05), self.navigation_input(), now=0.0)
        navigator.step(self.navigation_input(), now=0.1)
        raw = self.grid.raw.copy()
        raw[30, 40] = True
        changed = OccupancyGrid(raw, min_x=-3.0, min_y=-3.0, resolution_m=0.1,
                                inflation_radius_m=0.1)
        with self.assertRaises(PlanningError):
            navigator.update_map(changed, now=1.0)
        self.assertEqual(navigator.command, STOP)
        self.assertEqual(navigator.state, State.BLOCKED)

    def test_map_growth_preserves_dynamic_obstacle_expiry(self):
        cost_map = CostMap(self.grid, obstacle_ttl_seconds=2.0)
        point = Point2D(0.55, 0.55)
        cost_map.update_world_obstacles((point,), now=1.0)
        expanded = OccupancyGrid(np.zeros((80, 80), dtype=np.bool_), min_x=-4.0,
                                 min_y=-4.0, resolution_m=0.1, inflation_radius_m=0.1)
        cost_map.replace_static(expanded, now=2.0)
        self.assertFalse(cost_map.is_free(point))
        cost_map.update_world_obstacles((), now=3.1)
        self.assertTrue(cost_map.is_free(point))

    def test_inflated_start_has_finite_escape_cost_but_occupied_cell_does_not(self):
        cost_map = CostMap(self.grid, obstacle_ttl_seconds=2.0)
        obstacle = Point2D(0.05, 0.05)
        start = Point2D(0.15, 0.05)
        cost_map.update_world_obstacles((obstacle,), now=0.0)
        planner = GlobalPlanner(cost_map, Point2D(1.05, 0.05), self.config)
        self.assertFalse(cost_map.is_free(start))
        self.assertTrue(cost_map.is_raw_free(start))
        self.assertTrue(planner.plan(start))
        self.assertTrue(math.isfinite(planner.cost_to_go(start)))
        self.assertEqual(planner.progress_m(start, start), 0.0)
        self.assertEqual(planner.cost_to_go(obstacle), math.inf)
        self.assertEqual(planner.cost_to_go(Point2D(10.0, 0.0)), math.inf)

    def test_visible_obstacle_does_not_expire_when_start_is_inflated(self):
        navigator = Navigator(self.grid, self.config)
        pose = Pose2D(0.15, 0.05, 0.0)
        observed = NavigationInput(pose, STOP, (Point2D(-0.10, 0.0),))
        navigator.plan(Point2D(1.05, 0.05), observed, now=0.0)
        for now in (0.1, 1.1, 2.2, 3.3, 4.4):
            navigator.step(observed, now=now)
            self.assertFalse(navigator.cost_map.is_free(pose.to_point()))
            self.assertFalse(navigator.cost_map.is_raw_free(Point2D(0.05, 0.05)))
        # Expiration is still allowed once the obstacle is no longer observed.
        navigator.step(NavigationInput(pose, STOP, ()), now=6.5)
        self.assertTrue(navigator.cost_map.is_raw_free(Point2D(0.05, 0.05)))

    def test_visible_obstacle_at_goal_stops_then_times_out(self):
        config = replace(self.config, no_progress_timeout_seconds=3.0,
                         temporary_obstacle_ttl_seconds=0.5)
        navigator = Navigator(self.grid, config)
        pose = self.navigation_input()
        goal = Point2D(1.05, 0.05)
        navigator.plan(goal, pose, now=0.0)
        observed = NavigationInput(pose.pose, STOP, (goal,))
        for now in (0.1, 1.0, 2.0, 2.9):
            result = navigator.step(observed, now=now)
            self.assertEqual(result.state, State.NAVIGATING)
            self.assertEqual(result.command, STOP)
            self.assertFalse(navigator.cost_map.is_free(goal))
        result = navigator.step(observed, now=3.1)
        self.assertEqual(result.state, State.BLOCKED)
        self.assertEqual(result.command, STOP)

    def test_goal_clearing_before_timeout_resumes_after_obstacle_ttl(self):
        config = replace(self.config, no_progress_timeout_seconds=3.0,
                         temporary_obstacle_ttl_seconds=0.5)
        navigator = Navigator(self.grid, config)
        pose = self.navigation_input()
        goal = Point2D(1.05, 0.05)
        navigator.plan(goal, pose, now=0.0)
        navigator.step(NavigationInput(pose.pose, STOP, (goal,)), now=0.1)
        # A single missing observation must not immediately clear the obstacle.
        self.assertEqual(navigator.step(pose, now=0.3).command, STOP)
        result = navigator.step(pose, now=0.7)
        self.assertTrue(navigator.cost_map.is_free(goal))
        self.assertEqual(result.state, State.NAVIGATING)
        self.assertGreater(result.command.linear, 0.0)

    def test_local_ranking_preserves_first_command_on_exact_tie(self):
        cost_map = CostMap(self.grid, obstacle_ttl_seconds=2.0)
        local = LocalPlanner(cost_map, self.config)
        goal = Point2D(1.0, 0.0)
        planner = GlobalPlanner(cost_map, goal, self.config)
        pose = Pose2D(0.0, 0.0, 0.0)
        with (
            patch.object(local, "_window", return_value=((0.1,), (-0.2, 0.2))),
            patch.object(local, "_score", return_value=1.0),
        ):
            decision = local.command(pose, STOP, (goal,), (), planner, goal, False)
        self.assertTrue(decision.safe)
        self.assertEqual(decision.command, Velocity(0.1, -0.2))

    def test_inflated_start_can_turn_without_relaxing_collision_checks(self):
        cost_map = CostMap(self.grid, obstacle_ttl_seconds=2.0)
        cost_map.update_world_obstacles((Point2D(0.05, 0.05),), now=0.0)
        pose = Pose2D(0.15, 0.05, math.pi / 2.0)
        goal = Point2D(1.05, 0.05)
        planner = GlobalPlanner(cost_map, goal, self.config)
        local = LocalPlanner(cost_map, self.config)
        # Obstacle behind world X is on the left in this robot pose.
        obstacles = (Point2D(0.0, 0.10),)
        # Restrict this decision to rotations so both endpoints stay inflated.
        # Before the fix every candidate scored -inf and STOP won the tie.
        with patch.object(local, '_window', return_value=((0.0,), (-0.24, 0.0, 0.24))):
            decision = local.command(pose, STOP, planner.plan(pose.to_point()),
                                     obstacles, planner, goal, False)
        self.assertTrue(decision.safe)
        self.assertLess(decision.command.angular, 0.0)
        self.assertEqual(decision.command.linear, 0.0)
        self.assertTrue(local._safe_command(pose, decision.command, obstacles)[0])
        facing_obstacle = Pose2D(pose.x, pose.y, math.pi)
        self.assertFalse(local._safe_command(
            facing_obstacle, Velocity(0.10, 0.0), (Point2D(0.10, 0.0),)
        )[0])


if __name__ == "__main__":
    unittest.main()
