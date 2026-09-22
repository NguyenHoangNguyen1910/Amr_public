#!/usr/bin/env python3

"""Entrypoint for the ROS 2 navigation backend."""

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

from navigation.ros_adapter import NavigationNode


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
