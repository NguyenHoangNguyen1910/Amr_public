#!/usr/bin/env python3

"""Arbitrate velocity sources and enforce motion safety limits."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CmdVelArbiter(Node):
    """Select one safe velocity source for the robot base."""

    def __init__(self):
        super().__init__('cmd_vel_arbiter')

        self.declare_parameter('default_mode', 'manual')
        self.declare_parameter('manual_timeout', 0.35)
        self.declare_parameter('nav_timeout', 0.60)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('min_linear_x', -0.22)
        self.declare_parameter('max_linear_x', 0.30)
        self.declare_parameter('max_angular_z', 0.55)

        self.mode = str(self.get_parameter('default_mode').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.min_linear_x = min(0.0, float(self.get_parameter('min_linear_x').value))
        self.max_linear_x = max(0.0, float(self.get_parameter('max_linear_x').value))
        self.max_angular_z = abs(float(self.get_parameter('max_angular_z').value))
        self.emergency_stop = False
        self.manual_command = Twist()
        self.nav_command = Twist()
        self.manual_time = 0.0
        self.nav_time = 0.0

        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(
            Twist, '/web/cmd_vel_manual', self.manual_callback, 10)
        self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.create_subscription(
            String, '/web/control_mode', self.mode_callback, 10)
        self.create_subscription(
            Bool, '/web/emergency_stop', self.stop_callback, 10)
        self.create_subscription(
            Bool, '/web/emergency_reset', self.reset_callback, 10)

        rate = max(float(self.get_parameter('publish_rate').value), 5.0)
        self.timer = self.create_timer(1.0 / rate, self.publish_selected)
        self.get_logger().info('cmd_vel arbiter mode=%s' % self.mode)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, float(value)))

    def manual_callback(self, message):
        self.manual_command = message
        self.manual_time = time.monotonic()

    def nav_callback(self, message):
        self.nav_command = message
        self.nav_time = time.monotonic()

    def mode_callback(self, message):
        requested = str(message.data).strip().lower()
        if requested in ('manual', 'nav', 'stop'):
            if requested != self.mode:
                self.get_logger().info(
                    'control mode: %s -> %s' % (self.mode, requested))
            self.mode = requested

    def stop_callback(self, message):
        if message.data:
            self.emergency_stop = True
            self.get_logger().warning('emergency stop latched')

    def reset_callback(self, message):
        if message.data:
            self.emergency_stop = False
            self.get_logger().info('emergency stop reset')

    def publish_selected(self):
        selected = Twist()
        now = time.monotonic()

        if not self.emergency_stop:
            if self.mode == 'manual' and now - self.manual_time <= self.manual_timeout:
                selected = self.manual_command
            elif self.mode == 'nav' and now - self.nav_time <= self.nav_timeout:
                selected = self.nav_command

        selected.linear.x = self.clamp(
            selected.linear.x, self.min_linear_x, self.max_linear_x
        )
        selected.linear.y = 0.0
        selected.linear.z = 0.0
        selected.angular.x = 0.0
        selected.angular.y = 0.0
        selected.angular.z = self.clamp(
            selected.angular.z, -self.max_angular_z, self.max_angular_z
        )
        self.output.publish(selected)

    def publish_stop(self, count=5):
        stop = Twist()
        for _ in range(count):
            self.output.publish(stop)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelArbiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        try:
            node.destroy_node()
        except BaseException:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
