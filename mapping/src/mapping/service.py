"""REST API for the optional RTAB-Map mapping sidecar."""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import shutil
import time
from asyncio.subprocess import Process
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response

from navigation.map_io import OccupancyMap, load_occupancy_map

from config import (
    LOG_LEVEL,
    MAPPING_API_HOST,
    MAPPING_API_PORT,
    MAPPING_PROCESS_STOP_TIMEOUT_S,
    RTABMAP_CONFIG_PATH,
    RTABMAP_DB_DIRECTORY,
    RTABMAP_MAPPER_EXECUTABLE,
)


LOGGER = logging.getLogger("mapping.service")


class MappingWorker:
    """Own one RTAB-Map process and publish standard map artifacts."""

    def __init__(
        self,
        *,
        executable: str | None = None,
        db_dir: str | Path | None = None,
        config_path: str | Path | None = None,
        cwd: str | Path | None = None,
    ) -> None:
        self.executable = executable or RTABMAP_MAPPER_EXECUTABLE
        self.db_dir = Path(db_dir or RTABMAP_DB_DIRECTORY)
        self.config_path = Path(config_path or RTABMAP_CONFIG_PATH)
        self.cwd = str(cwd or Path.cwd())
        self._task: asyncio.Task | None = None
        self._process: Process | None = None
        self._stop_requested = False
        self._state = "STOPPED"
        self._error: str | None = None
        self._database_path: Path | None = None
        self._session_id: str | None = None
        self._artifacts: dict[str, str] = {}

    async def start(self) -> dict:
        if self._task is not None and not self._task.done():
            raise RuntimeError("A mapping session is already running")

        self.db_dir.mkdir(parents=True, exist_ok=True)
        self._session_id = f"scan_{time.time_ns()}"
        self._database_path = self.db_dir / f"{self._session_id}.db"
        self._artifacts = {}
        self._error = None
        self._state = "STARTING"
        self._stop_requested = False
        self._task = asyncio.create_task(self._run(), name="rtabmap-scan")
        return self.status()

    async def stop(self) -> dict:
        if self._task is None or self._task.done():
            return self.status()

        self._state = "STOPPING"
        self._stop_requested = True
        stopped_cleanly = True
        if self._process is not None:
            stopped_cleanly = await self._terminate(self._process)
        await asyncio.gather(self._task, return_exceptions=True)
        self._task = None
        self._process = None

        if not stopped_cleanly:
            self._set_error("mapper did not stop gracefully; artifacts were not published")
        elif self._error is None:
            self._publish_artifacts()

        if self._error is None and self._artifacts.get("image"):
            self._state = "SAVED"
        elif self._error is None:
            self._set_error("mapper stopped without producing a complete occupancy map")
        return self.status()

    async def shutdown(self) -> None:
        if self._task is not None and not self._task.done():
            await self.stop()

    async def _run(self) -> None:
        process: Process | None = None
        stderr_task: asyncio.Task | None = None
        try:
            environment = os.environ.copy()
            environment.update(
                {
                    "RTABMAP_DB_DIR": str(self.db_dir),
                    "RTABMAP_CONFIG": str(self.config_path),
                    "RTABMAP_SESSION_ID": self._session_id or "",
                }
            )
            process = await asyncio.create_subprocess_exec(
                self.executable,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
                cwd=self.cwd,
                env=environment,
            )
            self._process = process
            self._state = "WAITING_SENSOR"
            LOGGER.info("Started RTAB-Map mapper pid=%s session=%s", process.pid, self._session_id)
            stderr_task = asyncio.create_task(self._read_stderr(process), name="rtabmap-stderr")
            if self._stop_requested:
                await self._terminate(process)

            while process.returncode is None:
                if (
                    self._state == "WAITING_SENSOR"
                    and self._session_id is not None
                    and (self.db_dir / f"{self._session_id}.pgm").is_file()
                ):
                    self._state = "SCANNING"
                await asyncio.sleep(0.2)
            if not self._stop_requested:
                if process.returncode != 0:
                    raise RuntimeError(f"mapper exited with code {process.returncode}")
                raise RuntimeError("mapper exited before Stop & Save")
        except asyncio.CancelledError:
            raise
        except FileNotFoundError as error:
            self._set_error(f"mapper executable not found: {self.executable}")
            LOGGER.error("RTAB-Map mapper could not start: %s", error)
        except Exception as error:
            self._set_error(str(error))
            LOGGER.error("RTAB-Map mapper failed: %s", error)
            if process is not None and process.returncode is None:
                await self._terminate(process)
        finally:
            if stderr_task is not None:
                await asyncio.gather(stderr_task, return_exceptions=True)
            if process is not None and process.returncode is None:
                await process.wait()
            if process is not None:
                LOGGER.info("RTAB-Map mapper exited with code %s", process.returncode)
            if self._process is process:
                self._process = None

    async def _read_stderr(self, process: Process) -> None:
        if process.stderr is None:
            return
        while True:
            line = await process.stderr.readline()
            if not line:
                return
            message = line.decode(errors="replace").rstrip()
            LOGGER.info("[rtabmap_mapper] %s", message)

    @staticmethod
    async def _terminate(process: Process) -> bool:
        if process.returncode is not None:
            return True
        process.terminate()
        try:
            await asyncio.wait_for(
                process.wait(), timeout=MAPPING_PROCESS_STOP_TIMEOUT_S
            )
            return True
        except asyncio.TimeoutError:
            process.kill()
            await process.wait()
            return False

    def _set_error(self, message: str) -> None:
        self._state = "ERROR"
        self._error = message

    def live_map(self) -> tuple[OccupancyMap, int] | None:
        """Read the current scan artifacts when they form a stable snapshot."""
        if self._session_id is None:
            return None
        image_path = self.db_dir / f"{self._session_id}.pgm"
        yaml_path = self.db_dir / f"{self._session_id}.yaml"
        try:
            before = (
                image_path.stat().st_mtime_ns,
                image_path.stat().st_size,
                yaml_path.stat().st_mtime_ns,
                yaml_path.stat().st_size,
            )
            # The mapper writes the image before its metadata. Retry while the
            # pair is in that short update window.
            if before[2] < before[0]:
                return None
            occupancy = load_occupancy_map(image_path, yaml_path)
            after = (
                image_path.stat().st_mtime_ns,
                image_path.stat().st_size,
                yaml_path.stat().st_mtime_ns,
                yaml_path.stat().st_size,
            )
        except (OSError, ValueError):
            return None
        if before != after:
            return None
        return occupancy, before[0]

    def _publish_artifacts(self) -> None:
        if self._session_id is None:
            return
        base = self.db_dir / self._session_id
        artifacts = {
            "database": self._database_path or base.with_suffix(".db"),
            "image": base.with_suffix(".pgm"),
            "yaml": base.with_suffix(".yaml"),
        }
        missing = [str(path) for path in artifacts.values() if not path.is_file()]
        if missing:
            self._set_error(f"mapper did not produce required artifacts: {', '.join(missing)}")
            return

        self._artifacts = {name: str(path) for name, path in artifacts.items()}
        for suffix in (".pgm", ".yaml"):
            shutil.copy2(base.with_suffix(suffix), self.db_dir / f"scan_latest{suffix}")

    def status(self) -> dict[str, object | None]:
        return {
            "state": self._state,
            "session_id": self._session_id,
            "map_valid": bool(self._artifacts.get("image")),
            "database_path": str(self._database_path) if self._database_path else None,
            "artifact_paths": dict(self._artifacts),
            "error": self._error,
            "process_alive": self._process is not None and self._process.returncode is None,
            "process_pid": self._process.pid if self._process is not None else None,
        }


def create_app(worker: MappingWorker | None = None) -> FastAPI:
    mapping_worker = worker or MappingWorker()

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        try:
            yield
        finally:
            await mapping_worker.shutdown()

    app = FastAPI(title="Drone Mapping Service", lifespan=lifespan)
    app.add_middleware(
        CORSMiddleware,
        allow_origin_regex=r"^https?://[^/]+$",
        allow_credentials=False,
        allow_methods=["GET", "POST"],
        allow_headers=["*"],
        expose_headers=[
            "X-Map-Sequence",
            "X-Map-Width",
            "X-Map-Height",
            "X-Map-Resolution",
            "X-Map-Origin-East",
            "X-Map-Origin-North",
        ],
    )

    @app.exception_handler(RuntimeError)
    async def conflict(_: Request, error: RuntimeError) -> JSONResponse:
        return JSONResponse(status_code=409, content={"detail": str(error)})

    @app.get("/api/mapping/status")
    async def status() -> dict[str, Any]:
        return mapping_worker.status()

    @app.get("/api/mapping/map")
    async def map_snapshot() -> Response:
        result = mapping_worker.live_map()
        if result is None:
            return Response(status_code=204)
        occupancy, sequence = result
        return Response(
            content=occupancy.cells,
            media_type="application/octet-stream",
            headers={
                "X-Map-Sequence": str(sequence),
                "X-Map-Width": str(occupancy.width),
                "X-Map-Height": str(occupancy.height),
                "X-Map-Resolution": str(occupancy.resolution_m),
                "X-Map-Origin-East": str(occupancy.origin_east_m),
                "X-Map-Origin-North": str(occupancy.origin_north_m),
            },
        )

    @app.post("/api/mapping/start")
    async def start() -> dict[str, Any]:
        return {"ok": True, **await mapping_worker.start()}

    @app.post("/api/mapping/stop")
    async def stop() -> dict[str, Any]:
        return {"ok": True, **await mapping_worker.stop()}

    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=MAPPING_API_HOST)
    parser.add_argument("--port", type=int, default=MAPPING_API_PORT)
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(
        level=LOG_LEVEL,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    args = parse_args()
    import uvicorn

    uvicorn.run(create_app(), host=args.host, port=args.port, loop="asyncio")


if __name__ == "__main__":
    main()
