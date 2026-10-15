#!/usr/bin/env python3

"""Publish a canonical floating-point depth image for RTAB-Map.

The Astra driver publishes 16UC1 depth in millimetres.  RTAB-Map can consume
that format directly, but this adapter makes the unit and encoding explicit:
the output is always sensor_msgs/Image/32FC1 in metres.  It deliberately does
not apply the 600 mm preview threshold used by the MJPEG web stream.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthRtabmapAdapter(Node):
    """Convert raw Astra depth to a lossless ROS depth representation."""

    def __init__(self) -> None:
        super().__init__('depth_rtabmap_adapter')
        self.declare_parameter('input_topic', '/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/camera/depth/rtabmap')
        self.declare_parameter('uint16_scale_m', 0.001)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('max_output_rate_hz', 5.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.uint16_scale_m = float(self.get_parameter('uint16_scale_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.max_output_rate_hz = float(
            self.get_parameter('max_output_rate_hz').value)
        if self.uint16_scale_m <= 0.0:
            raise ValueError('uint16_scale_m must be positive')
        if self.max_depth_m <= 0.0:
            raise ValueError('max_depth_m must be positive')
        if self.max_output_rate_hz < 0.0:
            raise ValueError('max_output_rate_hz must be non-negative')

        self.publisher = self.create_publisher(
            Image, output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data)
        self._last_warning_at = 0.0
        self._last_output_stamp = None
        self._output_period = (
            1.0 / self.max_output_rate_hz
            if self.max_output_rate_hz > 0.0 else 0.0)
        self.get_logger().info(
            'RTAB-Map depth adapter: %s -> %s '
            '(output=32FC1 metres, max_rate=%.1f Hz)' %
            (input_topic, output_topic, self.max_output_rate_hz))

    def image_callback(self, message: Image) -> None:
        if self._output_period > 0.0:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            if (
                self._last_output_stamp is not None
                and stamp > self._last_output_stamp
                and stamp - self._last_output_stamp < self._output_period
            ):
                return
            self._last_output_stamp = stamp

        try:
            depth_m = self._decode(message)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                'Dropping depth frame: %s' % error,
                throttle_duration_sec=5.0,
            )
            return

        finite = np.isfinite(depth_m)
        valid = finite & (depth_m > 0.0) & (depth_m <= self.max_depth_m)
        depth_m = np.where(valid, depth_m, 0.0).astype('<f4', copy=False)

        valid_count = int(np.count_nonzero(valid))
        if valid_count == 0 and time.monotonic() - self._last_warning_at > 5.0:
            self._last_warning_at = time.monotonic()
            self.get_logger().warning(
                'Received a depth frame with no valid pixels; '
                'RTAB-Map cannot use this frame')

        output = Image()
        output.header = message.header
        output.height = message.height
        output.width = message.width
        output.encoding = '32FC1'
        output.is_bigendian = 0
        output.step = message.width * np.dtype('<f4').itemsize
        output.data = depth_m[:, :message.width].tobytes()
        self.publisher.publish(output)

    def _decode(self, message: Image) -> np.ndarray:
        encoding = message.encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            scalar_type = np.uint16
            scale_m = self.uint16_scale_m
        elif encoding in ('32fc1', '32fc'):
            scalar_type = np.float32
            scale_m = 1.0
        else:
            raise ValueError('unsupported depth encoding=%s' % message.encoding)

        dtype = np.dtype(scalar_type).newbyteorder(
            '>' if message.is_bigendian else '<')
        itemsize = dtype.itemsize
        if message.width <= 0 or message.height <= 0:
            raise ValueError('invalid image dimensions')
        if message.step < message.width * itemsize:
            raise ValueError(
                'step=%d is smaller than width=%d' %
                (message.step, message.width))
        expected_bytes = message.step * message.height
        if len(message.data) < expected_bytes:
            raise ValueError(
                'data has %d bytes, expected at least %d' %
                (len(message.data), expected_bytes))

        row_elements = message.step // itemsize
        values = np.frombuffer(
            message.data,
            dtype=dtype,
            count=row_elements * message.height,
        ).reshape(message.height, row_elements)[:, :message.width]
        return values.astype(np.float32, copy=False) * scale_m


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DepthRtabmapAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
