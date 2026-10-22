#!/usr/bin/env python3
"""Keep wheel measurements/timestamps; replace only unknown twist covariance."""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def wheel_covariance(covariance, linear_stddev):
    if not math.isfinite(linear_stddev) or linear_stddev <= 0:
        raise ValueError('wheel linear_stddev must be finite and positive')
    if any(value != 0.0 for value in covariance):
        return list(covariance)
    result = [0.0]*36
    for i in range(6):
        result[i*7] = linear_stddev**2 if i in (0, 1) else 1e6
    return result


class WheelCovarianceAdapter(Node):
    def __init__(self):
        super().__init__('wheel_covariance_adapter')
        self.stddev = self.declare_parameter('linear_stddev', 0.03).value
        wheel_covariance([0.0]*36, self.stddev)
        self.output = self.create_publisher(Odometry, '/wheel/odom_cov', 1)
        self.create_subscription(Odometry, '/odom_raw', self.callback, qos_profile_sensor_data)

    def callback(self, message):
        message.twist.covariance = wheel_covariance(message.twist.covariance, self.stddev)
        self.output.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = WheelCovarianceAdapter()
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
