"""ROS 2 adapter for the navigation backend."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import fields

import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from bringup.action import NavigateToPose
from .config import DEFAULT_CONFIG, NavigationConfig
from .global_planner import OccupancyGrid
from .navigator import Navigator
from .types import (
    NavigationInput,
    Point2D,
    Pose2D,
    SensorDataError,
    State,
    Velocity,
)

_FAILED_STATES = {State.CANCELLED.value, State.BLOCKED.value, State.ERROR.value}


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise SensorDataError(f"{label} is not numeric") from error
    if not math.isfinite(result):
        raise SensorDataError(f"{label} is not finite")
    return result


def _yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


def occupancy_grid_from_ros(
    message: RosOccupancyGrid,
    config: NavigationConfig,
    occupied_threshold: int,
) -> OccupancyGrid:
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = _finite(message.info.resolution, "map resolution")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("map dimensions and resolution must be positive")
    if len(message.data) != width * height:
        raise ValueError(
            f"map data length {len(message.data)} does not match {width * height} cells"
        )
    if abs(_yaw(message.info.origin.orientation)) > 1.0e-3:
        raise ValueError("rotated occupancy-grid origins are not supported")

    values = np.asarray(message.data, dtype=np.int16).reshape((height, width))
    return OccupancyGrid(
        (values < 0) | (values >= int(occupied_threshold)),
        min_x=_finite(message.info.origin.position.x, "map origin x"),
        min_y=_finite(message.info.origin.position.y, "map origin y"),
        resolution_m=resolution,
        inflation_radius_m=config.robot_radius_m + config.safety_margin_m,
    )


class NavigationNode(Node):
    """Bridge ROS topics/actions to the navigation core."""

    def __init__(self):
        super().__init__("navigation")
        self._callback_group = ReentrantCallbackGroup()
        self._sensor_lock = threading.Lock()
        self._mission_lock = threading.Lock()
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._scan_received_at: float | None = None
        self._odom_received_at: float | None = None
        self._latest_map: RosOccupancyGrid | None = None
        self._active_navigator: Navigator | None = None
        self._goal_reserved = False
        self._shutdown = False
        self._last_path: tuple[tuple[float, float], ...] = ()

        self.map_frame = str(self._parameter("map_frame", "map"))
        self.base_frame = str(self._parameter("base_frame", "base_footprint"))
        self.laser_frame = str(self._parameter("laser_frame", "laser_frame"))
        self.map_topic = str(self._parameter("map_topic", "/map"))
        self.scan_topic = str(self._parameter("scan_topic", "/scan"))
        self.odom_topic = str(self._parameter("odom_topic", "/odom"))
        self.cmd_vel_topic = str(self._parameter("cmd_vel_topic", "/cmd_vel_nav"))
        self.plan_topic = str(self._parameter("plan_topic", "/plan"))
        self.tf_timeout_seconds = max(
            0.01, float(self._parameter("tf_timeout_seconds", 0.10))
        )
        self.occupied_threshold = int(self._parameter("occupied_threshold", 65))

        self.config = NavigationConfig(
            **{
                field.name: self._parameter(field.name, getattr(DEFAULT_CONFIG, field.name))
                for field in fields(NavigationConfig)
            }
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._cmd_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self._plan_publisher = self.create_publisher(Path, self.plan_topic, 10)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            RosOccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            handle_accepted_callback=lambda handle: handle.execute(),
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            f"navigation ready: map={self.map_topic} scan={self.scan_topic} "
            f"odom={self.odom_topic} cmd_vel={self.cmd_vel_topic}"
        )

    def _parameter(self, name: str, default):
        return self.declare_parameter(name, default).value

    def _map_callback(self, message: RosOccupancyGrid) -> None:
        try:
            width = int(message.info.width)
            height = int(message.info.height)
            resolution = float(message.info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0:
                raise ValueError("map dimensions and resolution must be positive")
            if len(message.data) != width * height:
                raise ValueError("map data length does not match map dimensions")
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Ignoring invalid occupancy map: {error}",
                throttle_duration_sec=5.0,
            )
            return
        with self._mission_lock:
            self._latest_map = message

    def _scan_callback(self, message: LaserScan) -> None:
        with self._sensor_lock:
            self._latest_scan = message
            self._scan_received_at = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        with self._sensor_lock:
            self._latest_odom = message
            self._odom_received_at = time.monotonic()

    def navigation_input(self) -> NavigationInput:
        with self._sensor_lock:
            scan = self._latest_scan
            odometry = self._latest_odom
            scan_time = self._scan_received_at
            odom_time = self._odom_received_at

        now = time.monotonic()
        timeout = self.config.sensor_timeout_seconds
        if scan is None or scan_time is None:
            raise SensorDataError("LiDAR has not been received")
        if now - scan_time > timeout:
            raise SensorDataError("LiDAR is stale")
        if odometry is None or odom_time is None:
            raise SensorDataError("odometry has not been received")
        if now - odom_time > timeout:
            raise SensorDataError("odometry is stale")

        stamp = self._scan_stamp(scan)
        return NavigationInput(
            self._map_pose(stamp),
            Velocity(
                _finite(odometry.twist.twist.linear.x, "odometry linear velocity"),
                _finite(odometry.twist.twist.angular.z, "odometry angular velocity"),
            ),
            self._scan_points(scan, stamp),
        )

    def _map_pose(self, stamp: Time) -> Pose2D:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_seconds),
            ).transform
        except TransformException as error:
            raise SensorDataError(
                f"TF {self.map_frame}->{self.base_frame} unavailable"
            ) from error
        return Pose2D(
            _finite(transform.translation.x, "map pose x"),
            _finite(transform.translation.y, "map pose y"),
            _finite(_yaw(transform.rotation), "map pose yaw"),
        )

    @staticmethod
    def _scan_stamp(scan: LaserScan) -> Time:
        stamp = Time.from_msg(scan.header.stamp)
        return Time() if stamp.nanoseconds == 0 else stamp

    def _scan_points(self, scan: LaserScan, stamp: Time) -> tuple[Point2D, ...]:
        source_frame = scan.header.frame_id or self.laser_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.tf_timeout_seconds),
            ).transform
        except TransformException as error:
            raise SensorDataError(
                f"TF {source_frame}->{self.base_frame} unavailable for scan"
            ) from error

        range_min = _finite(scan.range_min, "LiDAR range_min")
        range_max = _finite(scan.range_max, "LiDAR range_max")
        angle_min = _finite(scan.angle_min, "LiDAR angle_min")
        angle_step = _finite(scan.angle_increment, "LiDAR angle_increment")
        if range_min <= 0.0 or range_max <= range_min:
            raise SensorDataError("LiDAR range limits are invalid")

        transform_yaw = _yaw(transform.rotation)
        cosine, sine = math.cos(transform_yaw), math.sin(transform_yaw)
        points = []
        for index, value in enumerate(scan.ranges):
            measured = float(value)
            if not math.isfinite(measured) or not range_min <= measured < range_max:
                continue
            angle = angle_min + index * angle_step
            laser_x = measured * math.cos(angle)
            laser_y = measured * math.sin(angle)
            points.append(
                Point2D(
                    transform.translation.x + laser_x * cosine - laser_y * sine,
                    transform.translation.y + laser_x * sine + laser_y * cosine,
                )
            )
        return tuple(points)

    def publish_velocity(self, linear: float, angular: float) -> dict[str, float]:
        linear = self._clamp_finite(linear, self.config.max_linear_velocity_mps)
        linear = max(self.config.min_linear_velocity_mps, linear)
        angular = self._clamp_finite(angular, self.config.max_angular_velocity_rps)
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self._cmd_publisher.publish(message)
        return {"linear": linear, "angular": angular}

    def publish_stop(self) -> None:
        self.publish_velocity(0.0, 0.0)

    @staticmethod
    def _clamp_finite(value: float, limit: float) -> float:
        value = float(value)
        return max(-abs(limit), min(abs(limit), value)) if math.isfinite(value) else 0.0

    def _goal_callback(self, request: NavigateToPose.Goal) -> GoalResponse:
        frame = request.pose.header.frame_id or self.map_frame
        if frame != self.map_frame or not self._valid_pose(request.pose):
            self.get_logger().warning("Rejecting goal with invalid pose or frame")
            return GoalResponse.REJECT
        with self._mission_lock:
            if self._goal_reserved or self._active_navigator is not None:
                self.get_logger().warning("Rejecting goal while navigation is active")
                return GoalResponse.REJECT
            if self._latest_map is None:
                self.get_logger().warning("Rejecting goal because no map is ready")
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        with self._mission_lock:
            navigator = self._active_navigator
        self.publish_stop()
        if navigator is not None:
            try:
                navigator.cancel()
            except Exception as error:
                self.get_logger().warning(f"Navigation cancel failed: {error}")
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        navigator: Navigator | None = None
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            with self._mission_lock:
                map_message = self._latest_map
            if map_message is None:
                raise RuntimeError("no occupancy map is available")
            navigator = Navigator(
                occupancy_grid_from_ros(map_message, self.config, self.occupied_threshold),
                self.config,
            )
            with self._mission_lock:
                self._active_navigator = navigator

            goal = goal_handle.request.pose.pose
            goal_point = Point2D(
                float(goal.position.x),
                float(goal.position.y),
                _yaw(goal.orientation),
            )
            navigation_input = self.navigation_input()
            navigator.plan(goal_point, navigation_input)
            self._publish_plan(navigator.path, goal_point.yaw)
            if goal_handle.is_cancel_requested:
                navigator.cancel()
                self.publish_stop()
                goal_handle.canceled()
                return NavigateToPose.Result()
            while not self._shutdown:
                result = navigator.step(self.navigation_input())
                self.publish_velocity(result.command.linear, result.command.angular)
                self._publish_plan(result.path, goal_point.yaw)
                if goal_handle.is_cancel_requested:
                    navigator.cancel()
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                if result.state is State.SUCCEEDED:
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                if result.state.value in _FAILED_STATES:
                    goal_handle.abort()
                    return NavigateToPose.Result()
                time.sleep(1.0 / self.config.control_hz)

            navigator.cancel()
            goal_handle.canceled()
            return NavigateToPose.Result()
        except Exception as error:
            self.publish_stop()
            self.get_logger().error(f"Navigation action failed: {error}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return NavigateToPose.Result()
        finally:
            self._publish_empty_plan()
            with self._mission_lock:
                if self._active_navigator is navigator:
                    self._active_navigator = None
                self._goal_reserved = False

    def _publish_plan(
        self, path: tuple[Point2D, ...], goal_yaw: float | None = None
    ) -> None:
        points = tuple((point.x, point.y) for point in path)
        if points == self._last_path:
            return
        self._last_path = points
        self._publish_path(points, goal_yaw)

    def _publish_empty_plan(self) -> None:
        self._last_path = ()
        self._publish_path(())

    def _publish_path(
        self,
        points: tuple[tuple[float, float], ...],
        goal_yaw: float | None = None,
    ) -> None:
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.poses = [
            self._pose_stamped(
                x,
                y,
                goal_yaw if goal_yaw is not None and index == len(points) - 1 else 0.0,
            )
            for index, (x, y) in enumerate(points)
        ]
        self._plan_publisher.publish(message)

    def _pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        message = PoseStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(x)
        message.pose.position.y = float(y)
        message.pose.orientation.z = math.sin(float(yaw) / 2.0)
        message.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return message

    @staticmethod
    def _valid_pose(message: PoseStamped) -> bool:
        pose = message.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        orientation = pose.orientation
        norm = math.sqrt(
            orientation.x**2
            + orientation.y**2
            + orientation.z**2
            + orientation.w**2
        )
        return norm >= 1.0e-6

    def destroy_node(self):
        self._shutdown = True
        with self._mission_lock:
            navigator = self._active_navigator
        if navigator is not None:
            navigator.cancel()
        self.publish_stop()
        self._action_server.destroy()
        return super().destroy_node()
