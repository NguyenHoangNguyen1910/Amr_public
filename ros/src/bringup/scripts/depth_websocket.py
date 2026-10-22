#!/usr/bin/env python3

"""Stream the newest depth frame over one binary WebSocket endpoint.

The ROS bridge remains responsible for pose, map, LiDAR, navigation and
commands.  This process only converts the depth image to a small 8-bit preview
and serves it through FastAPI/Uvicorn with WebSocket per-message deflate.
"""

import asyncio
import threading
import time

import numpy as np
import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LatestDepthFrame:
    """A latest-only frame buffer shared by the ROS and HTTP threads."""

    def __init__(self):
        self._lock = threading.Lock()
        self._payload = None
        self._sequence = 0
        self._updated_at = None
        self._width = 0
        self._height = 0

    def update(self, payload, width, height):
        with self._lock:
            self._payload = bytes(payload)
            self._sequence += 1
            self._updated_at = time.monotonic()
            self._width = int(width)
            self._height = int(height)

    def snapshot(self):
        with self._lock:
            if self._payload is None:
                return None
            return self._sequence, self._payload

    def health(self):
        with self._lock:
            age_ms = None
            if self._updated_at is not None:
                age_ms = round((time.monotonic() - self._updated_at) * 1000.0)
            return {
                'ready': self._payload is not None,
                'sequence': self._sequence,
                'frame_age_ms': age_ms,
                'width': self._width,
                'height': self._height,
            }


class DepthWebsocket(Node):
    """ROS depth subscriber plus the single depth WebSocket server."""

    def __init__(self):
        super().__init__('depth_websocket')
        self.declare_parameter('topic', '/camera/depth/image_raw')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8082)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('min_depth_m', 0.6)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('stream_hz', 30.0)

        self.topic = str(self.get_parameter('topic').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.width = max(1, int(self.get_parameter('width').value))
        self.height = max(1, int(self.get_parameter('height').value))
        self.min_depth_mm = float(self.get_parameter('min_depth_m').value) * 1000.0
        self.max_depth_mm = float(self.get_parameter('max_depth_m').value) * 1000.0
        self.stream_period = 1.0 / max(1.0, float(self.get_parameter('stream_hz').value))

        if self.max_depth_mm <= self.min_depth_mm:
            raise ValueError('max_depth_m must be greater than min_depth_m')

        self.frames = LatestDepthFrame()
        self.stop_event = threading.Event()
        self.app = FastAPI(title='robot_nav depth stream')
        self.server = None
        self.server_thread = None
        self._warned_encoding = set()
        self._install_routes()

        self.subscription = self.create_subscription(
            Image, self.topic, self.image_callback, qos_profile_sensor_data)

        self.server_thread = threading.Thread(
            target=self._run_server,
            name='depth-websocket',
            daemon=True,
        )
        self.server_thread.start()
        self.get_logger().info(
            'Depth WebSocket listening on ws://%s:%d/ws/depth '
            '(source: %s, permessage-deflate enabled)' % (
                self.host, self.port, self.topic))

    def _install_routes(self):
        @self.app.get('/healthz')
        async def healthz():
            return JSONResponse(self.frames.health())

        @self.app.websocket('/ws/depth')
        async def depth_stream(websocket: WebSocket):
            await websocket.accept()
            await websocket.send_json({
                'type': 'depth',
                'width': self.width,
                'height': self.height,
                'encoding': 'uint8-depth',
                'min_depth_m': self.min_depth_mm / 1000.0,
                'max_depth_m': self.max_depth_mm / 1000.0,
            })

            last_sequence = 0
            try:
                while not self.stop_event.is_set():
                    frame = self.frames.snapshot()
                    if frame is not None:
                        sequence, payload = frame
                        if sequence > last_sequence:
                            await websocket.send_bytes(payload)
                            last_sequence = sequence
                    await asyncio.sleep(self.stream_period)
            except WebSocketDisconnect:
                pass
            except (ConnectionError, RuntimeError):
                # The browser can close while a binary frame is being sent.
                pass

    def _run_server(self):
        config = uvicorn.Config(
            self.app,
            host=self.host,
            port=self.port,
            log_level='warning',
            access_log=False,
            ws_per_message_deflate=True,
        )
        self.server = uvicorn.Server(config)
        self.server.run()

    def image_callback(self, message):
        depth_mm = self._decode_depth_mm(message)
        if depth_mm is None:
            return

        sampled = self._resize_nearest(depth_mm)
        valid = np.isfinite(sampled)
        valid &= sampled >= self.min_depth_mm
        valid &= sampled <= self.max_depth_mm

        preview = np.zeros((self.height, self.width), dtype=np.uint8)
        # Near objects are bright, matching the depth preview used by
        # drone_sim. Zero is reserved for invalid/out-of-range pixels.
        normalized = 1.0 - (
            (sampled - self.min_depth_mm)
            / (self.max_depth_mm - self.min_depth_mm)
        )
        preview[valid] = np.clip(
            normalized[valid] * 254.0 + 1.0, 1.0, 255.0).astype(np.uint8)
        self.frames.update(preview.tobytes(), self.width, self.height)

    def _decode_depth_mm(self, message):
        encoding = str(message.encoding).strip().lower()
        if encoding in ('16uc1', 'mono16', '16uc'):
            dtype = np.dtype('>u2' if message.is_bigendian else '<u2')
            scale_mm = 1.0
        elif encoding in ('32fc1', '32fc'):
            dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
            scale_mm = 1000.0
        else:
            if encoding not in self._warned_encoding:
                self._warned_encoding.add(encoding)
                self.get_logger().warning(
                    'Unsupported depth encoding=%s on %s; expected 16UC1 or 32FC1' % (
                        message.encoding, self.topic))
            return None

        height = int(message.height)
        width = int(message.width)
        itemsize = dtype.itemsize
        step = int(message.step)
        required = step * height
        if height <= 0 or width <= 0 or step < width * itemsize:
            return None
        if len(message.data) < required:
            self.get_logger().warning('Depth image data is shorter than its step/height')
            return None

        try:
            rows = np.frombuffer(message.data[:required], dtype=dtype)
            rows = rows.reshape(height, step // itemsize)[:, :width]
            return rows.astype(np.float32, copy=False) * scale_mm
        except (ValueError, TypeError):
            return None

    def _resize_nearest(self, depth_mm):
        source_height, source_width = depth_mm.shape
        y_indices = np.linspace(
            0, source_height - 1, self.height).astype(np.intp)
        x_indices = np.linspace(
            0, source_width - 1, self.width).astype(np.intp)
        return depth_mm[np.ix_(y_indices, x_indices)]

    def close(self):
        self.stop_event.set()
        if self.server is not None:
            self.server.should_exit = True
        if self.server_thread is not None:
            self.server_thread.join(timeout=3.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthWebsocket()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
