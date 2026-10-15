"""ROS-independent camera transport: binary WebSocket and legacy MJPEG."""

import asyncio
import contextlib
import threading
import time

from aiohttp import web


class FrameStore:
    """Share one immutable, already-encoded JPEG across all clients."""

    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None
        self._sequence = 0
        self._updated_at = None
        self.notify = None

    def update(self, data):
        frame = bytes(data)
        with self._lock:
            self._frame = frame
            self._sequence += 1
            self._updated_at = time.monotonic()
        notify = self.notify
        if notify is not None:
            notify()

    def snapshot(self):
        with self._lock:
            return self._sequence, self._frame

    def health(self):
        with self._lock:
            age_ms = None
            if self._updated_at is not None:
                age_ms = round((time.monotonic() - self._updated_at) * 1000.0)
            return {'ready': self._frame is not None,
                    'sequence': self._sequence, 'frame_age_ms': age_ms}


class CameraStreamServer:
    """Run one event loop; slow clients retain only the latest pending frame."""

    def __init__(self, address, frame_store):
        self.address = address
        self.frame_store = frame_store
        self._ready = threading.Event()
        self._notify_lock = threading.Lock()
        self._notify_pending = False
        self._closing = False
        self._startup_error = None
        self._clients = set()
        self._websockets = set()
        self._thread = threading.Thread(target=self._run, name='camera-websocket',
                                        daemon=True)

    def start(self):
        self._thread.start()
        self._ready.wait()
        if self._startup_error is not None:
            self._thread.join()
            raise self._startup_error
        self.frame_store.notify = self._notify

    def _run(self):
        asyncio.run(self._serve())

    async def _serve(self):
        self._loop = asyncio.get_running_loop()
        self._stop = asyncio.Event()
        app = web.Application()
        app.router.add_get('/camera.ws', self._websocket)
        app.router.add_get('/camera.mjpg', self._mjpeg)
        app.router.add_get('/healthz', self._health)
        runner = web.AppRunner(app, access_log=None)
        try:
            await runner.setup()
            site = web.TCPSite(runner, *self.address, shutdown_timeout=5.0)
            await site.start()
            self.address = runner.addresses[0]
        except Exception as error:
            self._startup_error = error
            self._ready.set()
            await runner.cleanup()
            return
        self._ready.set()
        try:
            await self._stop.wait()
        finally:
            # Close active streams before aiohttp waits for listening sockets
            # to finish shutting down (also required on Python 3.12).
            await self._shutdown_clients(app)
            await runner.cleanup()

    def _notify(self):
        # ROS may publish faster than the network; never queue one callback
        # or one encoded image per incoming frame for a stalled event loop.
        with self._notify_lock:
            if self._closing or self._notify_pending:
                return
            self._notify_pending = True
            self._loop.call_soon_threadsafe(self._notify_clients)

    def _notify_clients(self):
        with self._notify_lock:
            self._notify_pending = False
        for event in self._clients:
            event.set()

    async def _frames(self, event):
        sequence = 0
        while not self._closing:
            event.clear()
            current, frame = self.frame_store.snapshot()
            if frame is not None and current != sequence:
                sequence = current
                yield frame
            else:
                await event.wait()

    async def _health(self, _request):
        return web.json_response(self.frame_store.health(), headers={
            'Cache-Control': 'no-store', 'Access-Control-Allow-Origin': '*'})

    async def _websocket(self, request):
        # aiohttp negotiates RFC 7692 and compresses each binary message.
        ws = web.WebSocketResponse(compress=True, heartbeat=20.0, timeout=2.0)
        await ws.prepare(request)
        event = asyncio.Event()
        self._clients.add(event)
        self._websockets.add(ws)

        async def send_frames():
            try:
                async for frame in self._frames(event):
                    await asyncio.wait_for(ws.send_bytes(frame), timeout=5.0)
            except (ConnectionError, asyncio.TimeoutError):
                await ws.close()

        sender = asyncio.create_task(send_frames())
        try:
            # Reading also handles close frames and ping/pong when ROS is idle.
            async for _message in ws:
                pass
        finally:
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await sender
            self._clients.discard(event)
            self._websockets.discard(ws)
        return ws

    async def _mjpeg(self, request):
        response = web.StreamResponse(headers={
            'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
            'Cache-Control': 'no-cache, no-store, must-revalidate',
            'Pragma': 'no-cache',
        })
        await response.prepare(request)
        event = asyncio.Event()
        self._clients.add(event)
        try:
            async for frame in self._frames(event):
                header = (b'--frame\r\nContent-Type: image/jpeg\r\n'
                          + ('Content-Length: %d\r\n\r\n' % len(frame)).encode('ascii'))
                await asyncio.wait_for(response.write(header + frame + b'\r\n'), 5.0)
        except (ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            self._clients.discard(event)
        return response

    async def _shutdown_clients(self, _app):
        self._notify_clients()
        await asyncio.gather(*(ws.close(code=1001, message=b'Server shutdown')
                               for ws in tuple(self._websockets)))

    def close(self):
        with self._notify_lock:
            if self._closing:
                return
            self._closing = True
        self.frame_store.notify = None
        self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            raise RuntimeError('Camera transport did not shut down')
