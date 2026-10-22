#!/usr/bin/env python3
"""Run alongside optional RTAB-Map VO, never publish TF or absolute EKF pose."""
import json
import math

import message_filters
import rclpy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rtabmap_msgs.msg import OdomInfo
from std_msgs.msg import String

from amr_perception.core import VOGate, fresh


class VOGateNode(Node):
    def __init__(self):
        super().__init__('vo_gate')
        p = lambda name, default: self.declare_parameter(name, default).value
        defaults = dict(recovery_samples=5, timeout=0.4, min_inliers=30,
                        min_ratio=0.4, max_speed=0.6, max_yaw_rate=1.0,
                        max_pose_step=0.25, variance_floor=0.04, variance_max=1.0)
        self.gate = VOGate(**{k: p(k, v) for k, v in defaults.items()})
        self.base_frame = p('base_frame', 'base_footprint')
        self.last_received_stamp = 0.0
        self.output = self.create_publisher(TwistWithCovarianceStamped, '/vo/twist', 1)
        self.status_pub = self.create_publisher(String, '/vo/status', 1)
        self.odom = message_filters.Subscriber(self, Odometry, '/vo/odom_raw',
                                              qos_profile=qos_profile_sensor_data)
        self.info = message_filters.Subscriber(self, OdomInfo, '/vo/odom_info',
                                              qos_profile=qos_profile_sensor_data)
        # Exact pairs: quality from a different frame must never admit a pose.
        self.sync = message_filters.TimeSynchronizer([self.odom, self.info], 3)
        self.sync.registerCallback(self.callback)
        self.create_timer(0.1, self.watchdog)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    def report(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.gate.state, reason=self.gate.reason,
            good_samples=self.gate.good, stamp=self.last_received_stamp))))

    def watchdog(self):
        if not fresh(self.last_received_stamp, self.now_seconds(), self.gate.timeout):
            self.gate.reject('VO missing/stale')
        self.report()

    def callback(self, odom, info):
        stamp = Time.from_msg(odom.header.stamp).nanoseconds*1e-9
        self.last_received_stamp = stamp
        if odom.child_frame_id != self.base_frame or odom.header.frame_id != 'vo_odom':
            self.gate.reject('unexpected VO frame')
            self.report()
            return
        q, pos, twist = odom.pose.pose.orientation, odom.pose.pose.position, odom.twist.twist
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not math.isfinite(norm) or abs(norm-1.0) > 0.02:
            self.gate.reject('invalid VO quaternion')
            self.report()
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        try:
            covariance = self.gate.admit(
                stamp=stamp, now=self.now_seconds(),
                velocity=(twist.linear.x, twist.linear.y, twist.angular.z),
                covariance=odom.twist.covariance,
                position=(pos.x, pos.y, pos.z), yaw=yaw, lost=info.lost,
                inliers=info.inliers, matches=info.matches, interval=info.interval,
                distance_travelled=info.distance_travelled)
        except (ValueError, TypeError) as error:
            self.gate.reject(str(error))
            covariance = None
        if covariance is not None:
            output = TwistWithCovarianceStamped()
            output.header.stamp = odom.header.stamp
            output.header.frame_id = self.base_frame
            output.twist.twist.linear.x = twist.linear.x
            output.twist.twist.linear.y = twist.linear.y
            output.twist.covariance = covariance
            self.output.publish(output)
        self.report()


def main(args=None):
    rclpy.init(args=args)
    node = VOGateNode()
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
