"""ROS 2 adapter for the vendored custom navigation backend."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import fields

import numpy as np
from builtin_interfaces.msg import Duration as DurationMessage
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
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

from .config import DEFAULT_CONFIG, NavigationConfig
from .global_planner import OccupancyGrid
from .helpers import SensorDataError
from .navigator import CustomNavigator
from .types import NavigationInput, Point2D, Pose2D, State, Velocity


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise SensorDataError(f"{label} is not numeric") from error
    if not math.isfinite(result):
        raise SensorDataError(f"{label} is not finite")
    return result


def _yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _rotate(rotation, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
    tx = 2.0 * (rotation.y * z - rotation.z * y)
    ty = 2.0 * (rotation.z * x - rotation.x * z)
    tz = 2.0 * (rotation.x * y - rotation.y * x)
    return (
        x + rotation.w * tx + rotation.y * tz - rotation.z * ty,
        y + rotation.w * ty + rotation.z * tx - rotation.x * tz,
        z + rotation.w * tz + rotation.x * ty - rotation.y * tx,
    )


def _duration(seconds: float) -> DurationMessage:
    seconds = max(0.0, float(seconds))
    result = DurationMessage()
    result.sec = int(seconds)
    result.nanosec = int((seconds - result.sec) * 1_000_000_000)
    return result


def occupancy_grid_from_ros(
    message: RosOccupancyGrid,
    config: NavigationConfig,
    occupied_threshold: int,
) -> OccupancyGrid:
    """Convert a ROS occupancy grid to the planner's axis-aligned grid."""

    width = int(message.info.width)
    height = int(message.info.height)
    resolution = _finite(message.info.resolution, "map resolution")
    if width <= 0 or height <= 0 or resolution <= 0.0:
        raise ValueError("map dimensions and resolution must be positive")

    expected = width * height
    if len(message.data) != expected:
        raise ValueError(
            f"map data length {len(message.data)} does not match {expected} cells"
        )

    origin_yaw = _yaw_from_quaternion(message.info.origin.orientation)
    if abs(origin_yaw) > 1.0e-3:
        raise ValueError("rotated occupancy-grid origins are not supported")

    values = np.asarray(message.data, dtype=np.int16).reshape((height, width))
    raw = (values < 0) | (values >= int(occupied_threshold))
    return OccupancyGrid(
        raw,
        min_x=_finite(message.info.origin.position.x, "map origin x"),
        min_y=_finite(message.info.origin.position.y, "map origin y"),
        resolution_m=resolution,
        inflation_radius_m=config.robot_radius_m + config.safety_margin_m,
    )


class RosNavigationTransport:
    """Provide the typed sensor/control contract expected by CustomNavigator."""

    def __init__(self, node: "CustomNavigationNode"):
        self.node = node

    def navigation_input(self) -> NavigationInput:
        with self.node._sensor_lock:
            scan = self.node._latest_scan
            odometry = self.node._latest_odom
            scan_received_at = self.node._scan_received_at
            odom_received_at = self.node._odom_received_at
            scan_sequence = self.node._scan_sequence

        now = time.monotonic()
        timeout = self.node.config.sensor_timeout_seconds
        if scan is None or scan_received_at is None:
            raise SensorDataError("LiDAR has not been received")
        if now - scan_received_at > timeout:
            raise SensorDataError("LiDAR is stale")
        if odometry is None or odom_received_at is None:
            raise SensorDataError("odometry has not been received")
        if now - odom_received_at > timeout:
            raise SensorDataError("odometry is stale")

        pose = self._map_pose()
        velocity = Velocity(
            _finite(odometry.twist.twist.linear.x, "odometry linear velocity"),
            _finite(odometry.twist.twist.angular.z, "odometry angular velocity"),
        )
        obstacles = self._scan_points(scan)
        return NavigationInput(pose, velocity, scan_sequence, obstacles)

    def set_velocity(self, linear: float, angular: float) -> object:
        return self.node.publish_velocity(linear, angular)

    def reset(self) -> bool:
        self.node.publish_stop()
        return True

    def _map_pose(self) -> Pose2D:
        try:
            transform = self.node.tf_buffer.lookup_transform(
                self.node.map_frame,
                self.node.base_frame,
                Time(),
                timeout=Duration(seconds=self.node.tf_timeout_seconds),
            )
        except TransformException as error:
            raise SensorDataError(
                f"TF {self.node.map_frame}->{self.node.base_frame} unavailable"
            ) from error

        translation = transform.transform.translation
        return Pose2D(
            _finite(translation.x, "map pose x"),
            _finite(translation.y, "map pose y"),
            _finite(_yaw_from_quaternion(transform.transform.rotation), "map pose yaw"),
        )

    def _scan_points(self, scan: LaserScan) -> tuple[Point2D, ...]:
        source_frame = scan.header.frame_id or self.node.laser_frame
        stamp = Time.from_msg(scan.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = Time()
        try:
            transform = self.node.tf_buffer.lookup_transform(
                self.node.base_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=self.node.tf_timeout_seconds),
            )
        except TransformException as error:
            raise SensorDataError(
                f"TF {source_frame}->{self.node.base_frame} unavailable for scan"
            ) from error

        range_min = _finite(scan.range_min, "LiDAR range_min")
        range_max = _finite(scan.range_max, "LiDAR range_max")
        angle_min = _finite(scan.angle_min, "LiDAR angle_min")
        angle_increment = _finite(scan.angle_increment, "LiDAR angle_increment")
        if range_min <= 0.0 or range_max <= range_min:
            raise SensorDataError("LiDAR range limits are invalid")

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        points: list[Point2D] = []
        for index, raw_range in enumerate(scan.ranges):
            measured = float(raw_range)
            if not math.isfinite(measured):
                continue
            if measured < range_min or measured >= range_max:
                continue
            angle = angle_min + index * angle_increment
            local_x = measured * math.cos(angle)
            local_y = measured * math.sin(angle)
            x, y, _ = _rotate(rotation, local_x, local_y)
            points.append(Point2D(x + translation.x, y + translation.y))
        return tuple(points)


class CustomNavigationNode(Node):
    """Expose CustomNavigator through the existing ROS/Nav2 action contract."""

    def __init__(self):
        super().__init__("custom_navigation")
        self._callback_group = ReentrantCallbackGroup()
        self._sensor_lock = threading.Lock()
        self._mission_lock = threading.RLock()
        self._latest_scan: LaserScan | None = None
        self._latest_odom: Odometry | None = None
        self._scan_received_at: float | None = None
        self._odom_received_at: float | None = None
        self._scan_sequence = 0
        self._latest_map: RosOccupancyGrid | None = None
        self._map_signature: tuple[object, ...] | None = None
        self._map_revision = 0
        self._active_navigator: CustomNavigator | None = None
        self._goal_reserved = False
        self._shutdown = False
        self._last_path: tuple[tuple[float, float], ...] = ()

        self.map_frame = self._declare_string("map_frame", "map")
        self.odom_frame = self._declare_string("odom_frame", "odom")
        self.base_frame = self._declare_string("base_frame", "base_footprint")
        self.laser_frame = self._declare_string("laser_frame", "laser_frame")
        self.map_topic = self._declare_string("map_topic", "/map")
        self.scan_topic = self._declare_string("scan_topic", "/scan")
        self.odom_topic = self._declare_string("odom_topic", "/odom")
        self.cmd_vel_topic = self._declare_string("cmd_vel_topic", "/cmd_vel_nav")
        self.plan_topic = self._declare_string("plan_topic", "/plan")
        self.tf_timeout_seconds = max(
            0.01, self._declare_float("tf_timeout_seconds", 0.10)
        )
        self.command_publish_hz = max(
            1.0, self._declare_float("command_publish_hz", DEFAULT_CONFIG.heartbeat_hz)
        )
        self.occupied_threshold = self._declare_int("occupied_threshold", 65)

        config_values = {}
        for field in fields(NavigationConfig):
            default = getattr(DEFAULT_CONFIG, field.name)
            value = self.declare_parameter(field.name, default).value
            config_values[field.name] = value
        config_values["heartbeat_hz"] = self.command_publish_hz
        self.config = NavigationConfig(**config_values)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._transport = RosNavigationTransport(self)
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
            handle_accepted_callback=self._handle_accepted_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "custom navigation ready: "
            f"map={self.map_topic} scan={self.scan_topic} "
            f"odom={self.odom_topic} cmd_vel={self.cmd_vel_topic}"
        )

    def _declare_string(self, name: str, default: str) -> str:
        return str(self.declare_parameter(name, default).value)

    def _declare_float(self, name: str, default: float) -> float:
        return float(self.declare_parameter(name, default).value)

    def _declare_int(self, name: str, default: int) -> int:
        return int(self.declare_parameter(name, default).value)

    def _map_callback(self, message: RosOccupancyGrid) -> None:
        try:
            width = int(message.info.width)
            height = int(message.info.height)
            resolution = float(message.info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0:
                raise ValueError("map dimensions and resolution must be positive")
            expected = width * height
            if len(message.data) != expected:
                raise ValueError(
                    f"map data length {len(message.data)} does not match {expected} cells"
                )
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                f"Ignoring invalid occupancy map: {error}",
                throttle_duration_sec=5.0,
            )
            return
        signature = (
            message.header.frame_id,
            int(message.info.width),
            int(message.info.height),
            float(message.info.resolution),
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
            float(message.info.origin.orientation.x),
            float(message.info.origin.orientation.y),
            float(message.info.origin.orientation.z),
            float(message.info.origin.orientation.w),
        )
        with self._mission_lock:
            self._latest_map = message
            self._map_signature = signature
            self._map_revision += 1

    def _scan_callback(self, message: LaserScan) -> None:
        with self._sensor_lock:
            self._latest_scan = message
            self._scan_received_at = time.monotonic()
            self._scan_sequence += 1

    def _odom_callback(self, message: Odometry) -> None:
        with self._sensor_lock:
            self._latest_odom = message
            self._odom_received_at = time.monotonic()

    def publish_velocity(self, linear: float, angular: float) -> dict[str, float]:
        linear = self._clamp_finite(linear, self.config.max_linear_velocity_mps)
        angular = self._clamp_finite(angular, self.config.max_angular_velocity_rps)
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self._cmd_publisher.publish(message)
        return {"linear": linear, "angular": angular}

    def publish_stop(self) -> None:
        for _ in range(3):
            self.publish_velocity(0.0, 0.0)

    @staticmethod
    def _clamp_finite(value: float, limit: float) -> float:
        parsed = float(value)
        if not math.isfinite(parsed):
            return 0.0
        return max(-abs(limit), min(abs(limit), parsed))

    def _goal_callback(self, goal_request: NavigateToPose.Goal) -> GoalResponse:
        pose = goal_request.pose
        frame = pose.header.frame_id or self.map_frame
        if frame != self.map_frame:
            self.get_logger().warning(f"Rejecting goal in frame {frame}")
            return GoalResponse.REJECT
        if not self._valid_pose(pose):
            self.get_logger().warning("Rejecting non-finite or invalid goal")
            return GoalResponse.REJECT
        with self._mission_lock:
            if self._goal_reserved or self._active_navigator is not None:
                self.get_logger().warning("Rejecting goal while navigation is active")
                return GoalResponse.REJECT
            if self._latest_map is None:
                self.get_logger().warning("Rejecting goal because no map is ready")
                return GoalResponse.REJECT
            try:
                self._transport.navigation_input()
            except SensorDataError as error:
                self.get_logger().warning(
                    f"Rejecting goal because navigation input is not ready: {error}"
                )
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

    @staticmethod
    def _handle_accepted_callback(goal_handle) -> None:
        goal_handle.execute()

    def _execute_callback(self, goal_handle):
        navigator: CustomNavigator | None = None
        active_map_signature: tuple[object, ...] | None = None
        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()

            with self._mission_lock:
                map_message = self._latest_map
                active_map_signature = self._map_signature
                if map_message is None:
                    raise RuntimeError("no occupancy map is available")
                static_map = occupancy_grid_from_ros(
                    map_message, self.config, self.occupied_threshold
                )
                navigator = CustomNavigator(
                    self._transport,
                    static_map=static_map,
                    config=self.config,
                )
                self._active_navigator = navigator

            goal = goal_handle.request.pose
            goal_point = Point2D(
                float(goal.pose.position.x),
                float(goal.pose.position.y),
                _yaw_from_quaternion(goal.pose.orientation),
            )
            navigator.plan(goal_point)
            self._publish_plan(navigator.status())
            if goal_handle.is_cancel_requested:
                navigator.cancel()
                goal_handle.canceled()
                return NavigateToPose.Result()
            navigator.start()

            started_at = time.monotonic()
            while not self._shutdown:
                navigator, active_map_signature = self._refresh_map_if_needed(
                    navigator,
                    active_map_signature,
                    goal_point,
                    goal_handle,
                )
                status = navigator.status()
                self._publish_plan(status)
                self._publish_feedback(goal_handle, status, time.monotonic() - started_at)
                if goal_handle.is_cancel_requested:
                    navigator.cancel()
                    goal_handle.canceled()
                    return NavigateToPose.Result()
                state = status["state"]
                if state == State.SUCCEEDED.value:
                    goal_handle.succeed()
                    return NavigateToPose.Result()
                if state in (
                    State.CANCELLED.value,
                    State.BLOCKED.value,
                    State.ERROR.value,
                ):
                    goal_handle.abort()
                    return NavigateToPose.Result()
                time.sleep(0.10)

            navigator.cancel()
            goal_handle.canceled()
            return NavigateToPose.Result()
        except Exception as error:
            self.publish_stop()
            self.get_logger().error(f"Custom navigation action failed: {error}")
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return NavigateToPose.Result()
        finally:
            if navigator is not None:
                navigator.close()
            self._publish_empty_plan()
            with self._mission_lock:
                if self._active_navigator is navigator:
                    self._active_navigator = None
            self._goal_reserved = False

    def _refresh_map_if_needed(
        self,
        navigator: CustomNavigator,
        active_map_signature: tuple[object, ...] | None,
        goal_point: Point2D,
        goal_handle,
    ) -> tuple[CustomNavigator, tuple[object, ...] | None]:
        """Replan on a SLAM map expansion without aborting the action.

        SLAM Toolbox can move the occupancy-grid origin or grow its dimensions
        while the robot is moving.  The current planner must not keep using a
        grid with stale world coordinates, but cancelling the action makes a
        normal map expansion look like a navigation failure to the UI.  Build
        and plan a replacement navigator first, then switch over atomically;
        the old navigator keeps the last safe command while this happens.
        """

        with self._mission_lock:
            map_message = self._latest_map
            latest_signature = self._map_signature
        if (
            map_message is None
            or latest_signature is None
            or latest_signature == active_map_signature
        ):
            return navigator, active_map_signature
        if goal_handle.is_cancel_requested or self._shutdown:
            return navigator, active_map_signature

        replacement: CustomNavigator | None = None
        try:
            static_map = occupancy_grid_from_ros(
                map_message, self.config, self.occupied_threshold
            )
            replacement = CustomNavigator(
                self._transport,
                static_map=static_map,
                config=self.config,
            )
            replacement.plan(goal_point)
        except Exception as error:
            if replacement is not None:
                replacement.close()
            self.get_logger().warning(
                f"Map update replan deferred: {error}",
                throttle_duration_sec=5.0,
            )
            return navigator, active_map_signature

        with self._mission_lock:
            map_is_current = self._map_signature == latest_signature
            if not map_is_current or goal_handle.is_cancel_requested or self._shutdown:
                replacement.close()
                return navigator, active_map_signature
            navigator.close()
            self._active_navigator = replacement

        replacement.start()
        self.get_logger().info(
            "Map metadata changed during navigation; replanned current goal"
        )
        return replacement, latest_signature

    def _publish_feedback(self, goal_handle, status, elapsed: float) -> None:
        feedback = NavigateToPose.Feedback()
        current = status.get("current_pose")
        if current is not None:
            feedback.current_pose = self._pose_stamped(
                current["x"], current["y"], current["yaw"]
            )
        distance = status.get("distance_to_goal_m")
        if distance is not None:
            feedback.distance_remaining = float(distance)
        feedback.navigation_time = _duration(elapsed)
        feedback.estimated_time_remaining = _duration(0.0)
        feedback.number_of_recoveries = 0
        goal_handle.publish_feedback(feedback)

    def _publish_plan(self, status) -> None:
        points = tuple(
            (float(point["x"]), float(point["y"]))
            for point in status.get("global_path", [])
        )
        if points == self._last_path:
            return
        self._last_path = points
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in points:
            message.poses.append(self._pose_stamped(x, y, 0.0))
        self._plan_publisher.publish(message)

    def _publish_empty_plan(self) -> None:
        self._last_path = ()
        message = Path()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
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
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        norm = math.sqrt(
            message.pose.orientation.x ** 2
            + message.pose.orientation.y ** 2
            + message.pose.orientation.z ** 2
            + message.pose.orientation.w ** 2
        )
        return norm >= 1.0e-6

    def destroy_node(self):
        self._shutdown = True
        with self._mission_lock:
            navigator = self._active_navigator
        if navigator is not None:
            navigator.close()
        self.publish_stop()
        self._action_server.destroy()
        return super().destroy_node()
