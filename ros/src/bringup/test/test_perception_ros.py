"""ROS regressions; run only in an isolated network/domain (tools/verify_architecture.sh)."""
import json
import math
import os
import subprocess
import time
import unittest

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
from std_msgs.msg import String

from cmd_vel_arbiter import CmdVelArbiter
from depth_obstacle_adapter import DepthObstacleAdapter
from motion_monitor import MotionMonitor
from navigation.ros_adapter import NavigationNode
from navigation.types import SensorDataError
from wheel_covariance_adapter import WheelCovarianceAdapter, wheel_covariance


class PerceptionROSTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        self.probe = self.add(Node('perception_test_probe'))

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def spin(self, duration=0.3):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.01)

    def tearDown(self):
        for node in reversed(self.nodes):
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()

    def test_wheel_unknown_covariance_only_and_stamp_preserved(self):
        node = self.add(WheelCovarianceAdapter())
        results = []
        self.probe.create_subscription(Odometry, '/wheel/odom_cov', results.append, 1)
        self.spin()
        message = Odometry()
        message.header.stamp = self.probe.get_clock().now().to_msg()
        message.child_frame_id = 'base_footprint'
        message.twist.twist.linear.x = 0.12
        node.callback(message)
        self.spin()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].header.stamp, message.header.stamp)
        self.assertEqual(results[0].twist.twist.linear.x, 0.12)
        self.assertAlmostEqual(results[0].twist.covariance[0], 0.03**2)
        known = list(np.eye(6).ravel())
        self.assertEqual(wheel_covariance(known, 0.03), known)

    def depth_image(self):
        image = Image()
        image.header.stamp = self.probe.get_clock().now().to_msg()
        image.header.frame_id = 'depth_optical'
        image.width = image.height = 81
        image.encoding, image.step = '16UC1', 162
        image.data = np.full((81, 81), 1000, dtype='<u2').tobytes()
        info = CameraInfo()
        info.header = image.header
        info.width = info.height = 81
        info.k = [80., 0., 40., 0., 80., 40., 0., 0., 1.]
        return image, info

    def test_depth_publishes_flat_scan_and_unknown_rear_with_zero_calibration(self):
        node = self.add(DepthObstacleAdapter())
        scans, clearances = [], []
        self.probe.create_subscription(LaserScan, '/depth/scan', scans.append, qos_profile_sensor_data)
        self.probe.create_subscription(LaserScan, '/depth/clearance', clearances.append, qos_profile_sensor_data)
        class TestBuffer:
            def lookup_transform(self, *args, **kwargs):
                assert args[2].nanoseconds == 0, 'rigid camera extrinsics must use latest TF'
                tf = TransformStamped()
                tf.transform.translation.z = 0.3
                q = tf.transform.rotation
                q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
                return tf
        node.buffer = TestBuffer()
        self.spin()
        image, info = self.depth_image()
        # Actual Astra registration metadata: K may be NaN; P is valid.
        info.k = [float('nan')]*9
        info.p = [80., 0., 40., 0., 0., 80., 40., 0., 0., 0., 1., 0.]
        node.info_callback(info)
        node.image_callback(image)
        self.spin()
        self.assertEqual(len(scans), 1)
        self.assertEqual(scans[0].header.stamp, image.header.stamp)
        self.assertEqual(scans[0].header.frame_id, 'base_footprint')
        self.assertEqual(scans[0].header, clearances[0].header)
        self.assertTrue(math.isnan(scans[0].ranges[0]))
        self.assertAlmostEqual(scans[0].ranges[90], 1.0)
        self.assertAlmostEqual(scans[0].range_min, 0.01)
        self.assertFalse(node.calibrated)  # No calibration movement interlock.

    def test_bad_camera_info_reports_failure_without_emitting_fresh_scan(self):
        node = self.add(DepthObstacleAdapter())
        statuses, scans = [], []
        self.probe.create_subscription(String, '/depth/status', statuses.append, 1)
        self.probe.create_subscription(LaserScan, '/depth/scan', scans.append, qos_profile_sensor_data)
        self.spin()
        image, info = self.depth_image()
        info.width = 1
        node.info_callback(info)
        node.image_callback(image)
        self.spin()
        self.assertFalse(scans)
        self.assertTrue(any('mismatch' in json.loads(x.data)['reason'] for x in statuses))

    def test_monitor_does_not_publish_or_block_cmd_vel_without_calibration(self):
        monitor = self.add(MotionMonitor())
        arbiter = self.add(CmdVelArbiter())
        outputs = []
        self.probe.create_subscription(Twist, '/cmd_vel', outputs.append, 10)
        self.spin()
        command = Twist()
        command.linear.x = 0.1
        arbiter.manual_callback(command)
        arbiter.publish_selected()
        monitor.check()
        self.spin(0.15)
        self.assertTrue(any(abs(x.linear.x-0.1) < 1e-6 for x in outputs))
        topics = self.probe.get_publisher_names_and_types_by_node('motion_monitor', '/')
        self.assertNotIn('/cmd_vel', [name for name, _ in topics])

    def test_navigation_rejects_fresh_delivery_of_old_source_data(self):
        nav = self.add(NavigationNode())
        scan, odom = LaserScan(), Odometry()
        scan.header.stamp.sec = odom.header.stamp.sec = 1
        nav._scan_callback(scan)
        nav._odom_callback(odom)
        with self.assertRaisesRegex(SensorDataError, 'source timestamp'):
            nav._fresh_sensor_snapshot()

    def test_monitor_reports_stale_wheel_even_with_fresh_ekf_output(self):
        monitor = self.add(MotionMonitor())
        statuses = []
        self.probe.create_subscription(String, '/motion/status', statuses.append, 1)
        self.spin()
        stamp = self.probe.get_clock().now().to_msg()
        wheel, filtered = Odometry(), Odometry()
        wheel.header.stamp.sec = 1
        filtered.header.stamp = stamp
        monitor.receive('wheel', wheel)
        monitor.receive('odom', filtered)
        monitor.check()
        self.spin(0.1)
        warnings = json.loads(statuses[-1].data)['warnings']
        self.assertIn('wheel missing/stale/future', warnings)
        self.assertNotIn('odom missing/stale/future', warnings)

    @unittest.skipUnless(os.environ.get('AMR_EKF_EXECUTABLE'), 'verified EKF executable required')
    def test_real_ekf_runs_without_vo_and_continues_after_vo_stops(self):
        self.add(WheelCovarianceAdapter())
        wheel = self.probe.create_publisher(Odometry, '/odom_raw', 10)
        imu = self.probe.create_publisher(Imu, '/imu/data_cov', 10)
        vo = self.probe.create_publisher(TwistWithCovarianceStamped, '/vo/twist', 10)
        output = []
        self.probe.create_subscription(Odometry, '/odom', output.append, 100)
        command = [os.environ['AMR_EKF_EXECUTABLE'], '--ros-args', '--params-file',
                   '/src/robot_localization/params/ekf.yaml', '-r', '/odometry/filtered:=/odom']
        with open('/verify/ekf-test.log', 'w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
            try:
                start = time.monotonic()
                phases = [[], [], []]
                while time.monotonic()-start < 5.0:
                    elapsed = time.monotonic()-start
                    phase = 0 if elapsed < 2 else (1 if elapsed < 3 else 2)
                    stamp = self.probe.get_clock().now().to_msg()
                    message = Odometry()
                    message.header.stamp, message.header.frame_id = stamp, 'wheel_odom'
                    message.child_frame_id = 'base_footprint'
                    message.twist.twist.linear.x = 0.1
                    wheel.publish(message)
                    message_imu = Imu()
                    message_imu.header.stamp, message_imu.header.frame_id = stamp, 'base_footprint'
                    message_imu.angular_velocity_covariance[8] = 0.0004
                    imu.publish(message_imu)
                    if phase == 1:
                        visual = TwistWithCovarianceStamped()
                        visual.header.stamp, visual.header.frame_id = stamp, 'base_footprint'
                        visual.twist.twist.linear.x = 0.1
                        visual.twist.covariance[0] = visual.twist.covariance[7] = 0.04
                        vo.publish(visual)
                    before = len(output)
                    self.spin(0.035)
                    phases[phase].extend(output[before:])
                self.assertIsNone(process.poll())
                self.assertGreater(len(phases[0]), 10, 'EKF must start without VO')
                self.assertGreater(len(phases[2]), 10, 'EKF must survive VO loss')
                positions = [m.pose.pose.position.x for m in output]
                self.assertGreater(positions[-1]-positions[0], 0.25)
                self.assertLess(max(abs(b-a) for a, b in zip(positions, positions[1:])), 0.03)
                self.assertTrue(all(m.header.frame_id == 'odom' for m in output))
            finally:
                process.terminate()
                process.wait(timeout=10)


if __name__ == '__main__':
    unittest.main()
