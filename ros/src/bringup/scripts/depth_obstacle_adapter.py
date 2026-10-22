#!/usr/bin/env python3
"""Project native/registered depth into a flat scan and observed-distance scan."""
import json
import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from amr_perception.core import DepthConfig, camera_intrinsics, decode_depth, depth_scan, fresh


class DepthObstacleAdapter(Node):
    def __init__(self):
        super().__init__('depth_obstacle_adapter')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.base_frame = p('base_frame', 'base_footprint')
        self.timeout = p('sensor_timeout', 0.4)
        self.info_timeout = p('camera_info_timeout', 2.0)
        self.max_rate = p('max_rate', 15.0)
        self.config = DepthConfig(**{k: p(k, v) for k, v in vars(DepthConfig()).items()})
        self.calibrated = p('calibration_confirmed', False)
        self.info = None
        self.last_processed = 0.0
        self.last_stamp = 0.0
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.scan_pub = self.create_publisher(LaserScan, '/depth/scan', qos_profile_sensor_data)
        self.clear_pub = self.create_publisher(LaserScan, '/depth/clearance', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/depth/status', 1)
        self.create_subscription(CameraInfo, p('camera_info_topic', '/camera/depth/camera_info'),
                                 self.info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, p('depth_topic', '/camera/depth/image_raw'),
                                 self.image_callback, qos_profile_sensor_data)
        self.create_timer(0.2, self.watchdog)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds*1e-9

    def info_callback(self, message):
        self.info = message

    def status(self, valid, reason, stamp=0.0, fraction=0.0):
        self.status_pub.publish(String(data=json.dumps(dict(
            valid=valid, reason=reason, stamp=stamp, valid_fraction=fraction,
            calibration_confirmed=self.calibrated))))

    def watchdog(self):
        if not fresh(self.last_stamp, self.now_seconds(), self.timeout):
            self.status(False, 'depth missing or stale', self.last_stamp)

    def image_callback(self, message):
        now = self.now_seconds()
        if now >= self.last_processed and now-self.last_processed < 1.0/self.max_rate:
            return
        self.last_processed = now
        stamp = Time.from_msg(message.header.stamp).nanoseconds*1e-9
        try:
            if not fresh(stamp, now, self.timeout) or stamp <= self.last_stamp:
                raise ValueError('depth timestamp stale/future/out of order')
            info = self.info
            if info is None:
                raise ValueError('CameraInfo missing')
            if (info.width != message.width or info.height != message.height
                    or not message.header.frame_id
                    or info.header.frame_id != message.header.frame_id):
                raise ValueError('depth/CameraInfo size or frame mismatch')
            info_stamp = Time.from_msg(info.header.stamp).nanoseconds*1e-9
            if not fresh(info_stamp, now, self.info_timeout):
                raise ValueError('CameraInfo stale/future')
            if (any(not math.isfinite(v) or abs(v) > 1e-8 for v in info.d)
                    or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                    or info.roi.x_offset or info.roi.y_offset):
                raise ValueError('depth must be rectified, unbinned and full-frame')
            # These are robot-fixed frames: the camera is rigidly mounted to
            # base_footprint. Astra republishes internal extrinsics on /tf at
            # a lower rate than images; their latest value is the same rigid
            # transform, and must not require a future image-time TF sample.
            # Keep the ORIGINAL image timestamp on the resulting scan so the
            # navigation pose is still evaluated at the acquisition time.
            tf = self.buffer.lookup_transform(self.base_frame, message.header.frame_id,
                                             Time(),
                                             timeout=Duration(seconds=0.03)).transform
            image = decode_depth(message.data, message.width, message.height,
                                 message.step, message.encoding, message.is_bigendian)
            t, q = tf.translation, tf.rotation
            obstacles, clearance, fraction = depth_scan(
                image, camera_intrinsics(info.k, info.p),
                (t.x, t.y, t.z), (q.x, q.y, q.z, q.w), self.config)
            for publisher, values in ((self.scan_pub, obstacles), (self.clear_pub, clearance)):
                scan = LaserScan()
                scan.header.stamp = message.header.stamp
                scan.header.frame_id = self.base_frame
                scan.angle_min = -math.pi
                scan.angle_increment = 2*math.pi/(self.config.bins-1)
                scan.angle_max = math.pi
                # Obstacle distance is measured from base, not the camera. Do
                # not discard close base-frame obstacles using optical Z limits.
                # The clearance stream carries a conservative blind-zone bound.
                scan.range_min = (0.01 if publisher is self.scan_pub else
                                  self.config.range_min+math.hypot(t.x, t.y))
                scan.range_max = self.config.range_max
                scan.scan_time = 1.0/self.max_rate
                scan.ranges = values.tolist()
                publisher.publish(scan)
            self.last_stamp = stamp
            valid = fraction >= self.config.min_valid_fraction
            self.status(valid, 'ok' if valid else 'insufficient valid depth', stamp, fraction)
        except (ValueError, TypeError, TransformException) as error:
            self.status(False, str(error), stamp)
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleAdapter()
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
