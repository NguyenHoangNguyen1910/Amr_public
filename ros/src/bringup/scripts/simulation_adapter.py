#!/usr/bin/env python3
"""Expose a saved occupancy map as a headless virtual ROS robot."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from bringup.srv import ResetSimulation

from simulation import DifferentialDriveSimulator, Pose2D, load_map


def _quaternion(yaw: float) -> tuple[float, float]:
    return math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class SimulationAdapter(Node):
    def __init__(self) -> None:
        super().__init__("simulation_adapter")
        self._lock = threading.Lock()
        self.map_frame = str(self._parameter("map_frame", "map"))
        self.odom_frame = str(self._parameter("odom_frame", "odom"))
        self.base_frame = str(self._parameter("base_frame", "base_footprint"))
        self.laser_frame = str(self._parameter("laser_frame", "laser_frame"))
        self.map_path = str(self._parameter("map_path", ""))
        if not self.map_path:
            raise ValueError("map_path is required")

        self.initial_pose = Pose2D(
            float(self._parameter("initial_x", -2.0)),
            float(self._parameter("initial_y", -0.5)),
            float(self._parameter("initial_yaw", 0.0)),
        )
        self.navigation_config_path = str(
            self._parameter("navigation_config_path", "")
        )
        self.robot_radius = self._load_robot_radius(self.navigation_config_path)
        self.max_linear = float(self._parameter("max_linear_velocity_mps", 0.30))
        self.max_angular = float(self._parameter("max_angular_velocity_rps", 0.55))
        self.command_timeout = float(self._parameter("command_timeout_seconds", 0.35))
        self.odom_hz = float(self._parameter("odom_hz", 30.0))
        self.scan_hz = float(self._parameter("scan_hz", 5.0))
        self.scan_count = int(self._parameter("scan_count", 360))
        self.scan_range_min = float(self._parameter("scan_range_min_m", 0.12))
        self.scan_range_max = float(self._parameter("scan_range_max_m", 3.5))
        self.lidar_x = float(self._parameter("lidar_x_m", -0.0046412))
        self.lidar_y = float(self._parameter("lidar_y_m", 0.0))
        self.lidar_yaw = float(self._parameter("lidar_yaw_rad", 0.0))

        self.grid_map = load_map(self.map_path)
        self.simulator = DifferentialDriveSimulator(
            self.grid_map, self.initial_pose, self.robot_radius
        )
        self._command = (0.0, 0.0)
        self._command_at = time.monotonic()
        self._last_step = time.monotonic()

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self.scan_publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.create_subscription(Twist, "/cmd_vel", self._command_callback, 10)
        self.create_service(
            ResetSimulation, "/simulation/reset", self._reset_callback
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transform()
        self._publish_map()
        self.create_timer(1.0 / self.odom_hz, self._update)
        self.create_timer(1.0 / self.scan_hz, self._publish_scan)
        self.create_timer(2.0, self._publish_map)
        self.get_logger().info(
            f"virtual robot ready: map={self.map_path} "
            f"size={self.grid_map.width}x{self.grid_map.height}"
        )

    def _parameter(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _load_robot_radius(config_path: str) -> float:
        if not config_path:
            raise ValueError("navigation_config_path is required")
        path = Path(config_path).expanduser()
        if not path.is_file():
            raise ValueError(f"navigation config not found: {path}")
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        try:
            value = float(
                document["navigation"]["ros__parameters"]["robot_radius_m"]
            )
        except (TypeError, KeyError, ValueError) as error:
            raise ValueError(
                f"robot_radius_m missing from navigation config: {path}"
            ) from error
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"robot_radius_m must be positive in {path}")
        return value

    def _command_callback(self, message: Twist) -> None:
        linear = max(-self.max_linear, min(self.max_linear, float(message.linear.x)))
        angular = max(-self.max_angular, min(self.max_angular, float(message.angular.z)))
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().warning("ignored non-finite velocity command")
            return
        with self._lock:
            self._command = (linear, angular)
            self._command_at = time.monotonic()

    def _update(self) -> None:
        now_monotonic = time.monotonic()
        dt = min(0.1, max(0.0, now_monotonic - self._last_step))
        self._last_step = now_monotonic
        with self._lock:
            linear, angular = self._command
            if now_monotonic - self._command_at > self.command_timeout:
                linear, angular = 0.0, 0.0
                self._command = (0.0, 0.0)
            pose = self.simulator.step(linear, angular, dt)
            measured_linear = self.simulator.linear_velocity
            measured_angular = self.simulator.angular_velocity
        stamp = self.get_clock().now().to_msg()
        z, w = _quaternion(pose.yaw)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = pose.x
        odometry.pose.pose.position.y = pose.y
        odometry.pose.pose.orientation.z = z
        odometry.pose.pose.orientation.w = w
        odometry.twist.twist.linear.x = measured_linear
        odometry.twist.twist.angular.z = measured_angular
        self.odom_publisher.publish(odometry)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.x
        transform.transform.translation.y = pose.y
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(transform)

    def _publish_scan(self) -> None:
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / self.scan_count
        with self._lock:
            ranges = self.simulator.scan(
                count=self.scan_count,
                angle_min=angle_min,
                angle_increment=angle_increment,
                range_min=self.scan_range_min,
                range_max=self.scan_range_max,
                sensor_x=self.lidar_x,
                sensor_y=self.lidar_y,
                sensor_yaw=self.lidar_yaw,
            )
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.laser_frame
        scan.angle_min = angle_min
        scan.angle_increment = angle_increment
        scan.angle_max = angle_min + (self.scan_count - 1) * angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.scan_hz
        scan.range_min = self.scan_range_min
        scan.range_max = self.scan_range_max
        scan.ranges = ranges
        self.scan_publisher.publish(scan)

    def _publish_map(self) -> None:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.info.resolution = self.grid_map.resolution
        message.info.width = self.grid_map.width
        message.info.height = self.grid_map.height
        message.info.origin.position.x = self.grid_map.origin_x
        message.info.origin.position.y = self.grid_map.origin_y
        z, w = _quaternion(self.grid_map.origin_yaw)
        message.info.origin.orientation.z = z
        message.info.origin.orientation.w = w
        message.data = list(self.grid_map.data)
        self.map_publisher.publish(message)

    def _publish_static_transform(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _reset_callback(self, request, response):
        values = (float(request.x), float(request.y), float(request.yaw))
        if not all(math.isfinite(value) for value in values):
            response.success = False
            response.message = "reset pose must be finite"
            return response
        pose = Pose2D(*values)
        if self.grid_map.collides(pose.x, pose.y, self.robot_radius):
            response.success = False
            response.message = "reset pose is occupied or too close to an obstacle"
            return response
        with self._lock:
            self.simulator = DifferentialDriveSimulator(
                self.grid_map, pose, self.robot_radius
            )
            self._command = (0.0, 0.0)
            self._command_at = time.monotonic()
            self._last_step = time.monotonic()
        response.success = True
        response.message = "simulation reset"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimulationAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
