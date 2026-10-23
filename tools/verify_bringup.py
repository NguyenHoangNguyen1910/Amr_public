"""Isolated launch check; cannot communicate with the hardware domain."""
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '83'
    rclpy.init()
    node = Node('bringup_verification')
    with open('/verify/bringup-launch.log', 'w') as log:
        process = subprocess.Popen([
            'ros2', 'launch', 'bringup', 'web.launch.py',
            'navigation_config:=/src/bringup/config/navigation.yaml',
            'use_depth_websocket:=false'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            expected = {'depth_obstacle_adapter', 'motion_monitor', 'wheel_covariance_adapter',
                        'ekf_filter_node', 'imu_covariance_adapter', 'navigation', 'cmd_vel_arbiter'}
            deadline = time.monotonic()+20
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if expected.issubset(set(node.get_node_names())):
                    break
            assert process.poll() is None, 'bringup exited'
            assert expected.issubset(set(node.get_node_names())), node.get_node_names()
            client = node.create_client(GetParameters, '/navigation/get_parameters')
            assert client.wait_for_service(timeout_sec=5)
            future = client.call_async(GetParameters.Request(names=[
                'scan_topic', 'min_linear_velocity_mps',
                'default_linear_velocity_mps', 'max_linear_velocity_mps',
                'max_angular_velocity_rps']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            values = future.result().values
            assert values[0].string_value == '/depth/scan'
            assert values[1].double_value == -0.35
            assert [value.double_value for value in values[2:]] == [0.35, 0.35, 0.40]
            arbiter = node.create_client(GetParameters, '/cmd_vel_arbiter/get_parameters')
            assert arbiter.wait_for_service(timeout_sec=5)
            future = arbiter.call_async(GetParameters.Request(names=[
                'min_linear_x', 'max_linear_x', 'max_angular_z',
                'manual_max_linear_x', 'manual_max_angular_z']))
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            assert [value.double_value for value in future.result().values] == [
                -0.35, 0.35, 0.40, 0.35, 0.40]
            publishers = node.get_publishers_info_by_topic('/cmd_vel')
            assert len(publishers) == 1, [p.node_name for p in publishers]
            assert publishers[0].node_name == 'cmd_vel_arbiter'
            web_subscriptions = node.get_subscriber_names_and_types_by_node('web_bridge', '/')
            assert '/depth/scan' in [topic for topic, _ in web_subscriptions]
            assert 'rgbd_odometry' not in node.get_node_names()
            print('Bringup launch passed: depth input, sole arbiter /cmd_vel publisher, no VO dependency')
        finally:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
