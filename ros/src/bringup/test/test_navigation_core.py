import math
import unittest
from dataclasses import replace

import numpy as np

from navigation.config import NavigationConfig
from navigation.global_planner import OccupancyGrid, PlanningError, GlobalPlanner, CostMap
from navigation.local_planner import LocalPlanner
from navigation.navigator import Navigator
from navigation.types import NavigationInput, Point2D, Pose2D, State, STOP


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


if __name__ == "__main__":
    unittest.main()
