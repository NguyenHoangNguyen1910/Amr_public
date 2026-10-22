#!/usr/bin/env python3
"""Advisory-only monitor. Never publishes or intercepts velocity commands."""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

from amr_perception.core import MotionConfig, fresh, supervise_motion


class MotionMonitor(Node):
    def __init__(self):
        super().__init__('motion_monitor')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.config = MotionConfig(**{k: p(k, v) for k, v in vars(MotionConfig()).items()})
        self.timeout = p('sensor_timeout', 0.6)
        self.samples = {}
        self.received = {}
        self.command = Twist()
        self.command_time = 0.0
        self.depth_status = None
        self.status_pub = self.create_publisher(String, '/motion/status', 1)
        for label, message_type, topic in (
                ('wheel', Odometry, '/odom_raw'), ('imu', Imu, '/imu/data_cov'),
                ('odom', Odometry, '/odom'), ('scan', LaserScan, '/depth/scan'),
                ('clearance', LaserScan, '/depth/clearance')):
            self.create_subscription(message_type, topic,
                                     lambda msg, key=label: self.receive(key, msg),
                                     qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command_callback, 1)
        self.create_subscription(String, '/depth/status', self.depth_callback, 1)
        self.create_timer(0.1, self.check)

    def receive(self, label, message):
        self.samples[label] = message
        self.received[label] = time.monotonic()

    def command_callback(self, message):
        self.command, self.command_time = message, time.monotonic()

    def depth_callback(self, message):
        try:
            self.depth_status = json.loads(message.data)
        except (ValueError, TypeError):
            self.depth_status = None

    def check(self):
        now = self.get_clock().now().nanoseconds*1e-9
        mono = time.monotonic()
        warnings = []
        for label in ('wheel', 'imu', 'odom', 'scan', 'clearance'):
            msg = self.samples.get(label)
            if (msg is None or mono-self.received[label] > self.timeout
                    or not fresh(Time.from_msg(msg.header.stamp).nanoseconds*1e-9,
                                 now, self.timeout)):
                warnings.append(label+' missing/stale/future')
        if not self.config.calibrated:
            warnings.append('camera/footprint calibration unconfirmed')
        if self.depth_status is None or not self.depth_status.get('valid', False):
            warnings.append('depth quality unavailable/insufficient')
        if mono-self.command_time > self.timeout:
            warnings.append('command stream missing/stale')
        if not warnings:
            scan, clear, odom = (self.samples[k] for k in ('scan', 'clearance', 'odom'))
            if (scan.header != clear.header or scan.header.frame_id != 'base_footprint'
                    or scan.angle_min != clear.angle_min
                    or scan.angle_increment != clear.angle_increment):
                warnings.append('scan/clearance geometry or timestamp mismatch')
            else:
                _, _, reason = supervise_motion(
                    self.command.linear.x, self.command.angular.z,
                    odom.twist.twist.linear.x, odom.twist.twist.angular.z,
                    scan.ranges, clear.ranges, scan.angle_min, scan.angle_increment,
                    clear.range_min, self.config)
                if reason not in ('clear', 'stopped'):
                    warnings.append(reason)
        self.status_pub.publish(String(data=json.dumps(dict(
            advisory_only=True, warnings=warnings, stamp=now))))


def main(args=None):
    rclpy.init(args=args)
    node = MotionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
