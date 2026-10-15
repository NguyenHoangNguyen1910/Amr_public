"""Real socket tests; run with scripts/ on PYTHONPATH, no ROS required."""

import asyncio
import struct
import threading
import unittest
import zlib

from aiohttp import ClientSession, ClientTimeout, WSMsgType

from camera_stream import CameraStreamServer, FrameStore


class CameraStreamTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.store = FrameStore()
        self.server = CameraStreamServer(('127.0.0.1', 0), self.store)
        self.server.start()
        self.url = 'http://127.0.0.1:%d' % self.server.address[1]
        self.client = ClientSession(timeout=ClientTimeout(total=5))

    async def asyncTearDown(self):
        await self.client.close()
        await asyncio.to_thread(self.server.close)

    async def test_wire_deflate_and_exact_binary_payload_across_frames(self):
        reader, writer = await asyncio.open_connection(*self.server.address)
        try:
            writer.write((
                'GET /camera.ws HTTP/1.1\r\nHost: localhost\r\n'
                'Upgrade: websocket\r\nConnection: Upgrade\r\n'
                'Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n'
                'Sec-WebSocket-Version: 13\r\n'
                'Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n\r\n'
            ).encode('ascii'))
            await writer.drain()
            headers = await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), 2)
            self.assertIn(b'101 Switching Protocols', headers)
            self.assertIn(b'Sec-WebSocket-Extensions: permessage-deflate', headers)
            inflater = zlib.decompressobj(-15)
            for payload in (b'\xff\xd8' + b'depth-frame-' * 2048 + b'\xff\xd9',
                            bytes(range(256)) * 100):
                self.store.update(payload)
                first, second = await asyncio.wait_for(reader.readexactly(2), 2)
                self.assertEqual(first, 0xC2, 'FIN + RSV1 (deflate) + binary opcode')
                self.assertEqual(second & 0x80, 0, 'Server frames must not be masked')
                size = second & 0x7F
                if size == 126:
                    size = struct.unpack('!H', await reader.readexactly(2))[0]
                elif size == 127:
                    size = struct.unpack('!Q', await reader.readexactly(8))[0]
                compressed = await asyncio.wait_for(reader.readexactly(size), 2)
                self.assertEqual(inflater.decompress(compressed + b'\x00\x00\xff\xff'),
                                 payload)
        finally:
            writer.close()
            await writer.wait_closed()

    async def test_multiple_clients_and_reconnect_receive_latest(self):
        self.store.update(b'first JPEG')
        async with self.client.ws_connect(self.url + '/camera.ws', compress=15) as first:
            async with self.client.ws_connect(self.url + '/camera.ws', compress=15) as second:
                for ws in (first, second):
                    self.assertGreater(ws.compress, 0)
                    message = await ws.receive(timeout=2)
                    self.assertEqual(message.type, WSMsgType.BINARY)
                    self.assertEqual(message.data, b'first JPEG')
                self.store.update(b'next JPEG')
                for ws in (first, second):
                    self.assertEqual((await ws.receive(timeout=2)).data, b'next JPEG')
        async with self.client.ws_connect(self.url + '/camera.ws', compress=15) as ws:
            self.assertEqual((await ws.receive(timeout=2)).data, b'next JPEG')

    async def test_burst_keeps_only_latest_when_event_loop_is_busy(self):
        async with self.client.ws_connect(self.url + '/camera.ws', compress=15) as ws:
            blocked = threading.Event()
            release = threading.Event()

            def stall():
                blocked.set()
                release.wait(3)

            self.server._loop.call_soon_threadsafe(stall)
            self.assertTrue(await asyncio.to_thread(blocked.wait, 2))
            try:
                for index in range(1000):
                    self.store.update(str(index).encode())
            finally:
                release.set()
            self.assertEqual((await ws.receive(timeout=2)).data, b'999')
            self.assertEqual(self.store.health()['sequence'], 1000)

    async def test_health_and_legacy_mjpeg_are_preserved(self):
        async with self.client.get(self.url + '/healthz') as response:
            self.assertEqual(await response.json(),
                             {'ready': False, 'sequence': 0, 'frame_age_ms': None})
            self.assertEqual(response.headers['Access-Control-Allow-Origin'], '*')
        frame = b'\xff\xd8sample\xff\xd9'
        self.store.update(frame)
        async with self.client.get(self.url + '/healthz') as response:
            status = await response.json()
            self.assertTrue(status['ready'])
            self.assertEqual(status['sequence'], 1)
        async with self.client.get(self.url + '/camera.mjpg') as response:
            self.assertIn('multipart/x-mixed-replace', response.headers['Content-Type'])
            header = b''
            while not header.endswith(b'\r\n\r\n'):
                header += await asyncio.wait_for(response.content.readline(), 2)
            self.assertIn(b'Content-Length: 10', header)
            self.assertEqual(await response.content.readexactly(len(frame)), frame)

    async def test_idle_client_can_close_and_server_can_shutdown(self):
        ws = await self.client.ws_connect(self.url + '/camera.ws', compress=15)
        await asyncio.wait_for(ws.close(), 2)
        async with self.client.ws_connect(self.url + '/camera.ws', compress=15) as active:
            closing = asyncio.create_task(asyncio.to_thread(self.server.close))
            self.assertEqual((await active.receive(timeout=3)).type, WSMsgType.CLOSE)
            await active.close()
            await asyncio.wait_for(closing, 4)

    async def test_bind_failure_is_reported(self):
        duplicate = CameraStreamServer(self.server.address, FrameStore())
        with self.assertRaises(OSError):
            duplicate.start()


if __name__ == '__main__':
    unittest.main()
