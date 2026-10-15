#!/usr/bin/env python3

"""Publish Astra RGB/depth frames and serve a lightweight depth WebSocket."""

import asyncio
import os
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
from primesense import openni2


class LatestDepthFrame:
    def __init__(self):
        self._lock = threading.Lock()
        self._sequence = 0
        self._payload = None
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


class AstraBridge(Node):
    def __init__(self):
        super().__init__('astra_bridge')

        self.declare_parameter('sdk', os.getenv(
            'OPENNI2_REDIST', '/workspace/src/openi_sdk'))
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8082)
        self.declare_parameter('stream_hz', 8.0)
        self.declare_parameter('preview_width', 320)
        self.declare_parameter('preview_height', 240)
        self.declare_parameter('max_depth_m', 5.0)

        self.sdk = str(self.get_parameter('sdk').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.stream_hz = max(1.0, float(self.get_parameter('stream_hz').value))
        self.preview_width = max(1, int(self.get_parameter('preview_width').value))
        self.preview_height = max(1, int(self.get_parameter('preview_height').value))
        self.max_depth_mm = max(
            1.0, float(self.get_parameter('max_depth_m').value) * 1000.0)

        self.color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, '/camera/depth/image_raw', qos_profile_sensor_data)

        openni2.initialize(self.sdk)
        self.device = openni2.Device.open_any()
        self.color = self.device.create_color_stream()
        self.depth = self.device.create_depth_stream()
        self.color.start()
        self.depth.start()

        for _ in range(10):
            self.color.read_frame()
            self.depth.read_frame()

        self.frames = LatestDepthFrame()
        self.stop_event = threading.Event()
        self.app = FastAPI(title='Astra depth stream')
        self.server = None
        self.server_thread = None
        self._install_routes()

        self.timer = self.create_timer(1.0 / self.stream_hz, self.capture)
        self.server_thread = threading.Thread(
            target=self._run_server,
            name='astra-depth-websocket',
            daemon=True,
        )
        self.server_thread.start()

        self.get_logger().info(
            'Astra ready: RGB=/camera/color/image_raw, '
            'depth=/camera/depth/image_raw, '
            'WebSocket=ws://%s:%d/ws/depth (permessage-deflate)' % (
                self.host, self.port))

    def _install_routes(self):
        @self.app.get('/healthz')
        async def healthz():
            return JSONResponse(self.frames.health())

        @self.app.websocket('/ws/depth')
        async def depth_stream(websocket: WebSocket):
            await websocket.accept()
            await websocket.send_json({
                'type': 'depth',
                'width': self.preview_width,
                'height': self.preview_height,
                'encoding': 'uint8-depth',
                'min_depth_m': 0.0,
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
                    await asyncio.sleep(1.0 / self.stream_hz)
            except (WebSocketDisconnect, ConnectionError, RuntimeError):
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

    def capture(self):
        try:
            color_frame = self.color.read_frame()
            depth_frame = self.depth.read_frame()
            stamp = self.get_clock().now().to_msg()

            color_raw = bytes(color_frame.get_buffer_as_uint8())
            depth_raw = bytes(depth_frame.get_buffer_as_uint16())

            self.color_pub.publish(self._image_message(
                color_raw,
                color_frame.width,
                color_frame.height,
                'rgb8',
                color_frame.width * 3,
                'camera_color_optical_frame',
                stamp,
            ))
            self.depth_pub.publish(self._image_message(
                depth_raw,
                depth_frame.width,
                depth_frame.height,
                '16UC1',
                depth_frame.width * 2,
                'camera_depth_optical_frame',
                stamp,
            ))

            preview = self._depth_preview(
                depth_raw, depth_frame.width, depth_frame.height)
            if preview is not None:
                self.frames.update(
                    preview, self.preview_width, self.preview_height)
        except Exception as error:
            # Some primesense error objects have a broken __str__ method.
            self.get_logger().error(
                'Astra capture failed: %s args=%r' % (
                    type(error).__name__, getattr(error, 'args', ())),
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _image_message(raw, width, height, encoding, step, frame_id, stamp):
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.height = int(height)
        message.width = int(width)
        message.encoding = encoding
        message.is_bigendian = 0
        message.step = int(step)
        message.data = raw
        return message

    def _depth_preview(self, raw, source_width, source_height):
        expected = int(source_width) * int(source_height) * 2
        if len(raw) != expected:
            return None

        depth = np.frombuffer(raw, dtype='<u2').reshape(
            int(source_height), int(source_width))
        y = np.linspace(
            0, source_height - 1, self.preview_height).astype(np.intp)
        x = np.linspace(
            0, source_width - 1, self.preview_width).astype(np.intp)
        sampled = depth[np.ix_(y, x)].astype(np.float32, copy=False)

        valid = (sampled > 0.0) & (sampled < self.max_depth_mm)
        preview = np.zeros(
            (self.preview_height, self.preview_width), dtype=np.uint8)
        # 1 is near/red, 255 is far/black; this matches read.py's palette.
        preview[valid] = np.clip(
            sampled[valid] / self.max_depth_mm * 254.0 + 1.0,
            1.0, 255.0,
        ).astype(np.uint8)
        return preview.tobytes()

    def close(self):
        self.stop_event.set()
        if self.server is not None:
            self.server.should_exit = True
        if self.server_thread is not None:
            self.server_thread.join(timeout=3.0)

        for stream in (self.color, self.depth):
            try:
                stream.stop()
                stream.close()
            except Exception:
                pass
        try:
            self.device.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AstraBridge()
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
