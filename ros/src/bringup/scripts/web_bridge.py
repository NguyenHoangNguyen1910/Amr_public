#!/usr/bin/env python3

"""Bridge robot state, laser data, and navigation to the web app."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformException, TransformListener
from bringup.action import NavigateToPose
from bringup.msg import ScanPoints


class WebBridge(Node):
    """Expose robot pose, laser points, and navigation to the web app."""

    def __init__(self):
        super().__init__('web_bridge')

        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('point_stride', 2)
        self.declare_parameter('max_points', 180)
        self.declare_parameter('transform_timeout', 0.05)
        self.declare_parameter('use_scan', False)
        self.declare_parameter('action_name', '/navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        publish_rate = max(float(self.get_parameter('publish_rate').value), 1.0)
        self.scan_publish_period = 1.0 / publish_rate
        self.point_stride = max(int(self.get_parameter('point_stride').value), 1)
        self.max_points = max(int(self.get_parameter('max_points').value), 1)
        self.transform_timeout = max(
            float(self.get_parameter('transform_timeout').value), 0.0)
        self.use_scan = bool(self.get_parameter('use_scan').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.last_scan_publish_time = 0.0
        self.scan_sequence = 0
        self.dropped_scan_count = 0
        self.goal_handle = None
        self.goal_sequence = 0
        self.localized = False
        self.last_map = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action_client = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('action_name').value))

        qos = 10
        self.pose_publisher = self.create_publisher(
            PoseStamped, '/robot_pose', qos)
        self.web_scan_points_publisher = self.create_publisher(
            ScanPoints, '/web/scan_points', qos)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.web_map_publisher = self.create_publisher(
            OccupancyGrid, '/web/map', map_qos)
        self.status_publisher = self.create_publisher(
            String, '/web/nav_status', qos)

        if self.use_scan:
            self.create_subscription(
                LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(
            PoseStamped, '/web/goal_pose', self.goal_callback, qos)
        self.create_subscription(
            Empty, '/web/cancel_navigation', self.cancel_callback, qos)
        self.create_subscription(
            Bool, '/web/emergency_stop', self.emergency_callback, qos)

        self.pose_timer = self.create_timer(0.05, self.publish_robot_pose)
        self.publish_status('IDLE', 'waiting for a goal')

    def publish_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time())
        except TransformException:
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.localized = True
        self.pose_publisher.publish(pose)

    def scan_callback(self, scan):
        now = time.monotonic()
        if now - self.last_scan_publish_time < self.scan_publish_period:
            return

        source_frame = scan.header.frame_id or 'laser_frame'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, source_frame, Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self.transform_timeout))
        except TransformException as error:
            self.dropped_scan_count += 1
            self.get_logger().warning(
                'Dropping scan: no TF at source timestamp (%s), dropped=%d' %
                (error, self.dropped_scan_count),
                throttle_duration_sec=5.0,
            )
            return

        web_points = self.laserscan_to_points(scan, transform)
        self.web_scan_points_publisher.publish(web_points)
        self.last_scan_publish_time = now

    def map_callback(self, message):
        if self.last_map is not None and self.maps_equal(self.last_map, message):
            return
        self.last_map = message
        self.web_map_publisher.publish(message)

    @staticmethod
    def maps_equal(first, second):
        first_info = first.info
        second_info = second.info
        first_origin = first_info.origin
        second_origin = second_info.origin
        return (
            first.header.frame_id == second.header.frame_id
            and first_info.width == second_info.width
            and first_info.height == second_info.height
            and first_info.resolution == second_info.resolution
            and first_origin.position.x == second_origin.position.x
            and first_origin.position.y == second_origin.position.y
            and first_origin.position.z == second_origin.position.z
            and first_origin.orientation.x == second_origin.orientation.x
            and first_origin.orientation.y == second_origin.orientation.y
            and first_origin.orientation.z == second_origin.orientation.z
            and first_origin.orientation.w == second_origin.orientation.w
            and first.data == second.data
        )

    @staticmethod
    def rotate_vector(rotation, x, y, z=0.0):
        tx = 2.0 * (rotation.y * z - rotation.z * y)
        ty = 2.0 * (rotation.z * x - rotation.x * z)
        tz = 2.0 * (rotation.x * y - rotation.y * x)
        return (
            x + rotation.w * tx + rotation.y * tz - rotation.z * ty,
            y + rotation.w * ty + rotation.z * tx - rotation.x * tz,
            z + rotation.w * tz + rotation.x * ty - rotation.y * tx,
        )

    def laserscan_to_points(self, scan, transform):
        web_points = ScanPoints()
        web_points.header.stamp = scan.header.stamp
        web_points.header.frame_id = self.map_frame
        self.scan_sequence += 1
        web_points.seq = self.scan_sequence

        step = max(
            self.point_stride, math.ceil(len(scan.ranges) / self.max_points))
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        for index in range(0, len(scan.ranges), step):
            distance = scan.ranges[index]
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            x, y, _ = self.rotate_vector(
                rotation,
                distance * math.cos(angle),
                distance * math.sin(angle),
            )
            web_points.points.extend((
                float(x + translation.x),
                float(y + translation.y),
            ))

        return web_points

    def publish_status(self, state, message='', **extra):
        payload = {'state': state, 'message': message}
        payload.update(extra)
        output = String()
        output.data = json.dumps(payload, separators=(',', ':'))
        self.status_publisher.publish(output)

    def valid_pose(self, message):
        if message.header.frame_id not in ('', self.map_frame):
            self.publish_status('ERROR', 'goal must use the map frame')
            return False

        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.publish_status('ERROR', 'goal contains a non-finite value')
            return False

        norm = math.sqrt(
            orientation.x ** 2 + orientation.y ** 2 +
            orientation.z ** 2 + orientation.w ** 2)
        if norm < 1.0e-6:
            self.publish_status('ERROR', 'goal orientation is invalid')
            return False
        return True

    def goal_callback(self, message):
        if not self.valid_pose(message):
            return

        if not self.localized:
            self.publish_status(
                'LOCALIZATION_REQUIRED',
                'waiting for SLAM pose before sending a goal')
            return

        if not message.header.frame_id:
            message.header.frame_id = self.map_frame

        self.goal_sequence += 1
        sequence = self.goal_sequence
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('ERROR', 'Navigation action server is not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = message
        self.publish_status('PLANNING', 'sending goal to navigation')
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self.goal_response_callback(result, sequence))

    def goal_response_callback(self, future, sequence):
        if sequence != self.goal_sequence:
            return
        try:
            handle = future.result()
        except Exception as error:
            self.publish_status('ERROR', 'Navigation rejected goal: %s' % error)
            return

        if not handle.accepted:
            self.publish_status('FAILED', 'Navigation rejected the goal')
            return

        self.goal_handle = handle
        self.publish_status('MOVING', 'robot is navigating')
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.result_callback(result, sequence))

    def result_callback(self, future, sequence):
        if sequence != self.goal_sequence:
            return
        try:
            status = future.result().status
        except Exception as error:
            self.goal_handle = None
            self.publish_status(
                'ERROR', 'failed to read navigation result: %s' % error)
            return

        self.goal_handle = None
        states = {
            GoalStatus.STATUS_SUCCEEDED: ('SUCCEEDED', 'goal reached'),
            GoalStatus.STATUS_CANCELED: ('CANCELED', 'goal canceled'),
            GoalStatus.STATUS_ABORTED: ('FAILED', 'Navigation aborted the goal'),
        }
        state, message = states.get(
            status, ('FAILED', 'Navigation finished with status %s' % status))
        self.publish_status(state, message)

    def cancel_callback(self, _message):
        self.goal_sequence += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.publish_status('CANCELED', 'navigation canceled')
        elif not self.localized:
            self.publish_status(
                'LOCALIZATION_REQUIRED',
                'waiting for SLAM pose before selecting GO')
        else:
            self.publish_status('IDLE', 'no active navigation goal')

    def emergency_callback(self, message):
        if message.data:
            self.cancel_callback(Empty())

    def close(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = WebBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.close()
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
