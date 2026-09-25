#!/usr/bin/env python3
"""End-to-end smoke tests for the saved-map 2D simulation.

Run this inside the ROS container after starting the stack with, for example:

    ros2 launch bringup web.launch.py \
      robot_backend:=simulation \
      sim_map:=/workspace/maps/warehouse_a.yaml \
      sim_initial_x:=0.005 sim_initial_y:=0.082 sim_initial_yaw:=0.001 \
      use_camera:=false

Each of ten fixed cases resets the simulator to a start pose and navigates to
one goal pose. The test uses the real ``/navigate_to_pose`` action and checks
``/odom`` at the goal.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from bringup.action import NavigateToPose
from bringup.srv import ResetSimulation
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from simulation import load_map
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


POSITION_TOLERANCE = 0.07
YAW_TOLERANCE = 0.10
ROBOT_RADIUS = 0.08


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(orientation) -> float:
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )


def message_stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


@dataclass(frozen=True)
class SmokeConfig:
    map_path: str


TEST_CASES = (
    ("case_01", (-2.688, -2.010, 0.0), (1.368, -3.423, 0.0)),
    ("case_02", (2.105, -2.475, math.pi), (-0.395, 0.025, math.pi / 2.0)),
    ("case_03", (0.105, -3.975, -math.pi / 2.0), (0.605, 0.025, -math.pi / 2.0)),
    ("case_04", (-2.895, -2.975, math.pi / 2.0), (-2.395, 0.525, 0.0)),
    ("case_05", (-0.895, -6.475, 0.0), (0.605, -2.975, math.pi)),
    ("case_06", (1.105, -0.975, -math.pi / 2.0), (-3.895, -0.475, math.pi / 2.0)),
    ("case_07", (-2.895, -5.475, math.pi), (-2.895, -1.975, 0.0)),
    ("case_08", (1.105, -4.475, math.pi / 2.0), (-0.895, -0.975, -math.pi / 2.0)),
    ("case_09", (-3.895, -2.975, 0.0), (-1.395, 0.025, math.pi)),
    ("case_10", (-0.895, -5.475, -math.pi / 2.0), (1.105, -1.975, math.pi / 2.0)),
)


class SimulationSmokeClient(Node):
    def __init__(self, config: SmokeConfig) -> None:
        super().__init__("simulation_smoke_client")
        self.config = config
        self.map_grid = load_map(config.map_path)
        self.action_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.reset_client = self.create_client(
            ResetSimulation, "/simulation/reset"
        )
        self.control_mode_publisher = self.create_publisher(
            String, "/web/control_mode", 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_odom: Odometry | None = None
        self.latest_scan: LaserScan | None = None
        self.latest_map: OccupancyGrid | None = None
        self.active_goal = None
        self.create_subscription(Odometry, "/odom", self._odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._map_callback, map_qos
        )

    def _odom_callback(self, message: Odometry) -> None:
        self.latest_odom = message

    def _scan_callback(self, message: LaserScan) -> None:
        self.latest_scan = message

    def _map_callback(self, message: OccupancyGrid) -> None:
        self.latest_map = message

    def wait_future(self, future, timeout: float, label: str):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"timed out waiting for {label}")
            rclpy.spin_once(self, timeout_sec=min(0.10, remaining))
        if not future.done():
            raise RuntimeError(f"ROS shutdown while waiting for {label}")
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def wait_until(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            if predicate():
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f"timed out waiting for {label}")
            rclpy.spin_once(self, timeout_sec=min(0.10, remaining))
        raise RuntimeError(f"ROS shutdown while waiting for {label}")

    def wait_for_stack(self, timeout: float) -> None:
        self.wait_until(
            lambda: self.action_client.wait_for_server(timeout_sec=0.0),
            timeout,
            "navigate_to_pose action server",
        )
        self.wait_until(
            lambda: self.reset_client.service_is_ready()
            or self.reset_client.wait_for_service(timeout_sec=0.0),
            timeout,
            "simulation reset service",
        )
        self.wait_until(
            lambda: self.latest_map is not None,
            timeout,
            "occupancy map",
        )
        self.wait_until(
            lambda: self.latest_odom is not None,
            timeout,
            "initial odometry",
        )
        self.wait_until(
            lambda: self.latest_scan is not None,
            timeout,
            "initial laser scan",
        )
        self.wait_until(self._tf_ready, timeout, "map to base_footprint TF")
        print("Simulation stack ready", flush=True)

    def set_control_mode(self, mode: str) -> None:
        message = String()
        message.data = mode
        for _ in range(3):
            self.control_mode_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset_simulation(
        self, x: float, y: float, yaw: float, timeout: float = 10.0
    ) -> None:
        if self.map_grid.collides(x, y, ROBOT_RADIUS):
            raise AssertionError(
                f"reset pose is occupied or too close to an obstacle: "
                f"({x}, {y})"
            )
        self.set_control_mode("stop")
        stopped_at = time.monotonic()
        old_odom = (
            message_stamp_ns(self.latest_odom) if self.latest_odom else -1
        )
        old_scan = (
            message_stamp_ns(self.latest_scan) if self.latest_scan else -1
        )
        self.wait_until(
            lambda: self.reset_client.service_is_ready()
            or self.reset_client.wait_for_service(timeout_sec=0.0),
            timeout,
            "simulation reset service",
        )
        request = ResetSimulation.Request()
        request.x = x
        request.y = y
        request.yaw = yaw
        response = self.wait_future(
            self.reset_client.call_async(request), timeout, "simulation reset"
        )
        if not response.success:
            raise AssertionError(f"simulation reset rejected: {response.message}")
        self.wait_until(
            lambda: self._fresh_odom(old_odom)
            and self._near_pose(x, y, yaw)
            and self._stationary(),
            timeout,
            "fresh stationary simulator reset pose",
        )
        self.wait_until(
            lambda: self._fresh_scan(old_scan), timeout, "fresh reset laser scan"
        )
        self.wait_until(self._tf_ready, timeout, "map to base_footprint TF")
        # The arbiter may still hold the previous navigation command. Its
        # default nav timeout is 0.60 s, so let that command expire before
        # enabling navigation for the next case.
        self.wait_until(
            lambda: time.monotonic() - stopped_at >= 0.70,
            timeout,
            "previous navigation command to expire",
        )
        self.set_control_mode("nav")

    def _fresh_odom(self, old_stamp_ns: int) -> bool:
        return self.latest_odom is not None and message_stamp_ns(
            self.latest_odom
        ) > old_stamp_ns

    def _fresh_scan(self, old_stamp_ns: int) -> bool:
        return self.latest_scan is not None and message_stamp_ns(
            self.latest_scan
        ) > old_stamp_ns

    def _tf_ready(self) -> bool:
        return self.tf_buffer.can_transform(
            "map", "base_footprint", Time(), timeout=Duration(seconds=0.0)
        )

    def _near_pose(self, x: float, y: float, yaw: float) -> bool:
        if self.latest_odom is None:
            return False
        pose = self.latest_odom.pose.pose
        position_ok = math.hypot(
            pose.position.x - x,
            pose.position.y - y,
        ) <= 0.05
        yaw_ok = abs(
            normalize_angle(
                quaternion_yaw(pose.orientation) - yaw
            )
        ) <= 0.05
        return position_ok and yaw_ok

    def _stationary(self) -> bool:
        if self.latest_odom is None:
            return False
        velocity = self.latest_odom.twist.twist
        return abs(velocity.linear.x) <= 0.02 and abs(velocity.angular.z) <= 0.03

    @staticmethod
    def make_goal(
        x: float, y: float, yaw: float
    ) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def send_goal(self, goal: NavigateToPose.Goal, timeout: float = 10.0):
        goal_handle = self.wait_future(
            self.action_client.send_goal_async(goal), timeout, "goal response"
        )
        if goal_handle.accepted:
            self.active_goal = goal_handle
        return goal_handle

    def wait_result(self, goal_handle, timeout: float):
        result = self.wait_future(
            goal_handle.get_result_async(), timeout, "navigation result"
        )
        if self.active_goal is goal_handle:
            self.active_goal = None
        return result

    def cancel_active_goal(self) -> None:
        goal_handle = self.active_goal
        if goal_handle is None:
            return
        try:
            self.wait_future(
                goal_handle.cancel_goal_async(), 5.0, "goal cancellation"
            )
            self.wait_future(
                goal_handle.get_result_async(), 10.0, "canceled navigation result"
            )
        except Exception:
            pass
        finally:
            self.active_goal = None

    def run_leg(self, name: str, x: float, y: float, yaw: float) -> None:
        print(f"  RUN {name}: goal=({x:.3f}, {y:.3f}, {yaw:.3f})", flush=True)
        if self.map_grid.collides(x, y, ROBOT_RADIUS):
            raise AssertionError(f"{name}: goal is occupied in {self.config.map_path}")
        goal_handle = self.send_goal(self.make_goal(x, y, yaw))
        if not goal_handle.accepted:
            raise AssertionError(f"{name}: goal was unexpectedly rejected")

        try:
            result = self.wait_result(goal_handle, 180.0)
        except TimeoutError as error:
            pose = self.latest_odom.pose.pose if self.latest_odom else None
            pose_text = "unknown" if pose is None else (
                f"({pose.position.x:.3f}, {pose.position.y:.3f}, "
                f"{quaternion_yaw(pose.orientation):.3f})"
            )
            raise TimeoutError(f"{name}: {error}; last_pose={pose_text}") from error
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            pose = self.latest_odom.pose.pose if self.latest_odom else None
            pose_text = "unknown"
            if pose is not None:
                pose_text = (
                    f"({pose.position.x:.3f}, {pose.position.y:.3f}, "
                    f"{quaternion_yaw(pose.orientation):.3f})"
                )
            raise AssertionError(
                f"{name}: expected SUCCEEDED, got status {result.status}; "
                f"final_pose={pose_text}"
            )

        def reached_goal() -> bool:
            if self.latest_odom is None:
                return False
            pose = self.latest_odom.pose.pose
            distance = math.hypot(pose.position.x - x, pose.position.y - y)
            yaw_error = abs(normalize_angle(quaternion_yaw(pose.orientation) - yaw))
            linear = abs(self.latest_odom.twist.twist.linear.x)
            angular = abs(self.latest_odom.twist.twist.angular.z)
            return (
                distance <= POSITION_TOLERANCE
                and yaw_error <= YAW_TOLERANCE
                and linear <= 0.04
                and angular <= 0.05
            )

        self.wait_until(reached_goal, 3.0, f"{name} final pose")
        pose = self.latest_odom.pose.pose
        distance = math.hypot(pose.position.x - x, pose.position.y - y)
        yaw_error = abs(normalize_angle(quaternion_yaw(pose.orientation) - yaw))
        print(
            f"  PASS {name}: "
            f"pose=({pose.position.x:.3f}, {pose.position.y:.3f}, "
            f"{quaternion_yaw(pose.orientation):.3f}) "
            f"error=({distance:.3f}m, {yaw_error:.3f}rad)",
            flush=True,
        )

    def run_test_case(
        self,
        index: int,
        name: str,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
    ) -> None:
        print(
            f"RUN {index}/10 {name}: "
            f"start=({start[0]:.3f}, {start[1]:.3f}, {start[2]:.3f}) "
            f"-> goal=({goal[0]:.3f}, {goal[1]:.3f}, {goal[2]:.3f})",
            flush=True,
        )
        self.reset_simulation(*start)
        self.run_leg(f"{name}/start_to_goal", *goal)
        print(f"PASS {index}/10 {name}", flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map",
        default=os.environ.get("SMOKE_MAP", "/workspace/maps/warehouse_a.yaml"),
        help="map path visible inside the ROS container",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = SmokeConfig(map_path=args.map)
    rclpy.init()
    node = None
    try:
        node = SimulationSmokeClient(config)
        node.wait_for_stack(20.0)
        failures = []
        for index, (name, start, goal) in enumerate(TEST_CASES, 1):
            try:
                node.run_test_case(index, name, start, goal)
            except Exception as error:
                print(f"FAIL {name}: {error}", file=sys.stderr, flush=True)
                failures.append(f"{name}: {error}")
                node.cancel_active_goal()
        if failures:
            raise AssertionError(f"{len(failures)}/10 cases failed: " + "; ".join(failures))
        print("Simulation smoke test passed: 10/10 start -> goal cases.")
        return 0
    except Exception as error:
        print(f"Simulation smoke test FAILED: {error}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.cancel_active_goal()
            node.set_control_mode("manual")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
