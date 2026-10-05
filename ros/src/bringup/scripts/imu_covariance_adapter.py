#!/usr/bin/env python3
"""Supply explicit uncertainty only when the IMU declares it unknown (all zeros).

Measurements, timestamps, frames, known covariances and the -1 unavailable
sentinel are preserved. Defaults are engineering starting values, not a sensor
calibration. This node does not filter, resample or downsample measurements.
"""

import math


def covariance_or_default(covariance, standard_deviation):
    if not math.isfinite(standard_deviation) or standard_deviation <= 0:
        raise ValueError("IMU standard deviation must be finite and positive")
    if any(value != 0.0 for value in covariance):
        return covariance
    variance = standard_deviation * standard_deviation
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    rclpy.init(args=args)
    node = Node("imu_covariance_adapter")
    orientation_stddev = float(node.declare_parameter("orientation_stddev", 0.10).value)
    angular_stddev = float(node.declare_parameter("angular_velocity_stddev", 0.02).value)
    for value in (orientation_stddev, angular_stddev):
        covariance_or_default([0.0] * 9, value)
    publisher = node.create_publisher(Imu, "/imu/data_cov", qos_profile_sensor_data)

    def callback(message):
        message.orientation_covariance = covariance_or_default(
            message.orientation_covariance, orientation_stddev
        )
        message.angular_velocity_covariance = covariance_or_default(
            message.angular_velocity_covariance, angular_stddev
        )
        publisher.publish(message)

    node.create_subscription(Imu, "/imu/data", callback, qos_profile_sensor_data)
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
