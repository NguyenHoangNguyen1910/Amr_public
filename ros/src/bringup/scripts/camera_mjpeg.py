#!/usr/bin/env python3

"""Serve ROS camera JPEG frames over WebSocket with permessage-deflate."""

from camera_stream import CameraStreamServer, FrameStore

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraMjpeg(Node):
    """Bridge an RGB or depth ROS image to the camera WebSocket endpoint."""

    def __init__(self):
        super().__init__('camera_mjpeg')
        self.declare_parameter('topic', '/camera/depth/image_raw')
        self.declare_parameter('mode', 'depth')
        self.declare_parameter('min_depth_mm', 600.0)
        self.declare_parameter('max_depth_mm', 6000.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8081)

        topic = str(self.get_parameter('topic').value)
        self.mode = str(self.get_parameter('mode').value).strip().lower()
        if self.mode not in ('depth', 'compressed'):
            self.get_logger().warning(
                'Unknown camera mode=%s; using depth' % self.mode)
            self.mode = 'depth'
        self.min_depth_mm = float(self.get_parameter('min_depth_mm').value)
        self.max_depth_mm = float(self.get_parameter('max_depth_mm').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        host = str(self.get_parameter('host').value)
        port = int(self.get_parameter('port').value)

        self.frame_store = FrameStore()
        self.stream_server = CameraStreamServer((host, port), self.frame_store)
        self.stream_server.start()

        message_type = Image if self.mode == 'depth' else CompressedImage
        self.subscription = self.create_subscription(
            message_type, topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            'Camera WebSocket listening on ws://%s:%d/camera.ws (permessage-deflate) '
            '(source: %s, mode: %s)' % (host, port, topic, self.mode))

    def image_callback(self, message):
        if self.mode == 'depth':
            frame = self.depth_to_jpeg(message)
            if frame is not None:
                self.frame_store.update(frame)
            return

        if message.format and 'jpeg' not in message.format.lower() \
                and 'jpg' not in message.format.lower():
            self.get_logger().warn(
                'Expected JPEG CompressedImage, got format=%s' % message.format,
                throttle_duration_sec=10.0,
            )
            return
        self.frame_store.update(message.data)

    def depth_to_jpeg(self, message):
        """Convert Astra's raw depth image in millimetres to a colour JPEG."""
        encoding = message.encoding.lower()
        if encoding in ('16uc1', 'mono16'):
            scalar_type = np.uint16
            scale_to_mm = 1.0
        elif encoding in ('32fc1', '32fc'):
            scalar_type = np.float32
            scale_to_mm = 1000.0
        else:
            self.get_logger().warning(
                'Unsupported depth encoding=%s' % message.encoding,
                throttle_duration_sec=10.0,
            )
            return None

        try:
            endian = '>' if message.is_bigendian else '<'
            dtype = np.dtype(scalar_type).newbyteorder(endian)
            row_elements = message.step // dtype.itemsize
            if row_elements < message.width:
                raise ValueError(
                    'step=%d is smaller than image width=%d' %
                    (message.step, message.width))
            expected_bytes = message.step * message.height
            if len(message.data) < expected_bytes:
                raise ValueError(
                    'data has %d bytes, expected at least %d' %
                    (len(message.data), expected_bytes))
            depth = np.frombuffer(
                message.data,
                dtype=dtype,
                count=row_elements * message.height,
            ).reshape(message.height, row_elements)[:, :message.width]
            depth_mm = depth.astype(np.float32, copy=False) * scale_to_mm

            valid = (
                np.isfinite(depth_mm)
                & (depth_mm >= self.min_depth_mm)
                & (depth_mm <= self.max_depth_mm)
            )
            normalized = np.zeros(depth_mm.shape, dtype=np.uint8)
            span = max(self.max_depth_mm - self.min_depth_mm, 1.0)
            normalized[valid] = np.clip(
                (depth_mm[valid] - self.min_depth_mm) * 255.0 / span,
                0.0,
                255.0,
            ).astype(np.uint8)

            frame = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
            frame[~valid] = 0
            quality = min(max(self.jpeg_quality, 1), 100)
            ok, encoded = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if not ok:
                raise ValueError('OpenCV could not encode the depth frame')
            return encoded.tobytes()
        except (ValueError, TypeError) as error:
            self.get_logger().warning(
                'Could not convert depth image: %s' % error,
                throttle_duration_sec=10.0,
            )
            return None

    def close(self):
        self.stream_server.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraMjpeg()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            if rclpy.ok():
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
