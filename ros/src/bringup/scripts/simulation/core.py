"""ROS-independent differential-drive and occupancy-grid simulation core."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Sequence

import yaml


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class GridMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: tuple[int, ...]
    occupied_threshold: int = 65

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("map data length does not match its dimensions")

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        dx = x - self.origin_x
        dy = y - self.origin_y
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        return (
            math.floor(local_x / self.resolution),
            math.floor(local_y / self.resolution),
        )

    def value(self, grid_x: int, grid_y: int) -> int:
        if not (0 <= grid_x < self.width and 0 <= grid_y < self.height):
            return 100
        return self.data[grid_y * self.width + grid_x]

    def blocked(self, grid_x: int, grid_y: int) -> bool:
        value = self.value(grid_x, grid_y)
        return value < 0 or value >= self.occupied_threshold

    def collides(self, x: float, y: float, radius: float) -> bool:
        min_grid = self.world_to_grid(x - radius, y - radius)
        max_grid = self.world_to_grid(x + radius, y + radius)
        # Saved maps normally have zero origin yaw. Check a conservative box for
        # rotated maps as well; false positives are safer than crossing a wall.
        pad = 1 if abs(self.origin_yaw) > 1e-9 else 0
        for grid_y in range(min_grid[1] - pad, max_grid[1] + pad + 1):
            for grid_x in range(min_grid[0] - pad, max_grid[0] + pad + 1):
                if not self.blocked(grid_x, grid_y):
                    continue
                cell_x, cell_y = self.grid_cell_center(grid_x, grid_y)
                half = self.resolution * (0.5 + pad)
                nearest_x = max(cell_x - half, min(x, cell_x + half))
                nearest_y = max(cell_y - half, min(y, cell_y + half))
                if math.hypot(x - nearest_x, y - nearest_y) <= radius:
                    return True
        return False

    def grid_cell_center(self, grid_x: int, grid_y: int) -> tuple[float, float]:
        local_x = (grid_x + 0.5) * self.resolution
        local_y = (grid_y + 0.5) * self.resolution
        cos_yaw = math.cos(self.origin_yaw)
        sin_yaw = math.sin(self.origin_yaw)
        return (
            self.origin_x + cos_yaw * local_x - sin_yaw * local_y,
            self.origin_y + sin_yaw * local_x + cos_yaw * local_y,
        )

    def raycast(
        self,
        x: float,
        y: float,
        angle: float,
        range_min: float,
        range_max: float,
    ) -> float:
        step = min(0.02, self.resolution * 0.25)
        distance = max(0.0, range_min)
        cos_angle = math.cos(angle)
        sin_angle = math.sin(angle)
        while distance <= range_max:
            grid_x, grid_y = self.world_to_grid(
                x + distance * cos_angle,
                y + distance * sin_angle,
            )
            if self.blocked(grid_x, grid_y):
                return distance
            distance += step
        return math.inf


class DifferentialDriveSimulator:
    """Integrate velocity commands and stop translation at occupied cells."""

    def __init__(self, grid_map: GridMap, pose: Pose2D, robot_radius: float):
        if robot_radius <= 0.0:
            raise ValueError("robot radius must be positive")
        if grid_map.collides(pose.x, pose.y, robot_radius):
            raise ValueError("initial pose is occupied or outside the map")
        self.grid_map = grid_map
        self.pose = pose
        self.robot_radius = robot_radius
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def step(self, linear: float, angular: float, dt: float) -> Pose2D:
        if not all(math.isfinite(value) for value in (linear, angular, dt)):
            raise ValueError("simulation inputs must be finite")
        if dt <= 0.0:
            return self.pose
        yaw_mid = self.pose.yaw + angular * dt * 0.5
        candidate_x = self.pose.x + linear * math.cos(yaw_mid) * dt
        candidate_y = self.pose.y + linear * math.sin(yaw_mid) * dt
        candidate_yaw = _normalize_angle(self.pose.yaw + angular * dt)
        if self.grid_map.collides(candidate_x, candidate_y, self.robot_radius):
            self.linear_velocity = 0.0
            self.angular_velocity = angular
            self.pose = Pose2D(self.pose.x, self.pose.y, candidate_yaw)
        else:
            self.linear_velocity = linear
            self.angular_velocity = angular
            self.pose = Pose2D(candidate_x, candidate_y, candidate_yaw)
        return self.pose

    def scan(
        self,
        *,
        count: int,
        angle_min: float,
        angle_increment: float,
        range_min: float,
        range_max: float,
        sensor_x: float = 0.0,
        sensor_y: float = 0.0,
        sensor_yaw: float = 0.0,
    ) -> list[float]:
        if count <= 0:
            raise ValueError("scan count must be positive")
        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        origin_x = self.pose.x + cos_yaw * sensor_x - sin_yaw * sensor_y
        origin_y = self.pose.y + sin_yaw * sensor_x + cos_yaw * sensor_y
        return [
            self.grid_map.raycast(
                origin_x,
                origin_y,
                self.pose.yaw + sensor_yaw + angle_min + index * angle_increment,
                range_min,
                range_max,
            )
            for index in range(count)
        ]


def load_map(path: str | Path) -> GridMap:
    map_path = Path(path).expanduser().resolve()
    suffix = map_path.suffix.lower()
    if suffix == ".json":
        return _load_simulator_json(map_path)
    if suffix in (".yaml", ".yml"):
        return _load_ros_map(map_path)
    raise ValueError(f"unsupported map format: {map_path.suffix}")


def _load_simulator_json(path: Path) -> GridMap:
    document = json.loads(path.read_text(encoding="utf-8"))
    resolution = float(document["resolution_m"])
    bounds = document["bounds"]
    origin_x = float(bounds["min_x"])
    origin_y = float(bounds["min_y"])
    width = math.ceil((float(bounds["max_x"]) - origin_x) / resolution)
    height = math.ceil((float(bounds["max_y"]) - origin_y) / resolution)
    data = [0] * (width * height)
    for obstacle in document.get("obstacles", []):
        if obstacle.get("type") != "circle":
            raise ValueError(f"unsupported JSON obstacle type: {obstacle.get('type')}")
        center_x = float(obstacle["x"])
        center_y = float(obstacle["y"])
        radius = float(obstacle["radius"])
        for grid_y in range(height):
            cell_y = origin_y + (grid_y + 0.5) * resolution
            if abs(cell_y - center_y) > radius + resolution:
                continue
            for grid_x in range(width):
                cell_x = origin_x + (grid_x + 0.5) * resolution
                if math.hypot(cell_x - center_x, cell_y - center_y) <= radius:
                    data[grid_y * width + grid_x] = 100
    return GridMap(width, height, resolution, origin_x, origin_y, 0.0, tuple(data))


def _load_ros_map(path: Path) -> GridMap:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("map YAML must contain an object")
    image_path = Path(str(document["image"]))
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    width, height, max_value, pixels = _read_pgm(image_path)
    negate = bool(int(document.get("negate", 0)))
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    free_threshold = float(document.get("free_thresh", 0.196))
    data: list[int] = []
    # PGM rows start at the top; OccupancyGrid rows start at the map origin.
    for source_y in range(height - 1, -1, -1):
        for source_x in range(width):
            pixel = pixels[source_y * width + source_x]
            probability = pixel / max_value if negate else (max_value - pixel) / max_value
            if probability > occupied_threshold:
                data.append(100)
            elif probability < free_threshold:
                data.append(0)
            else:
                data.append(-1)
    origin: Sequence[float] = document.get("origin", (0.0, 0.0, 0.0))
    if len(origin) < 3:
        raise ValueError("map origin must contain x, y and yaw")
    return GridMap(
        width,
        height,
        float(document["resolution"]),
        float(origin[0]),
        float(origin[1]),
        float(origin[2]),
        tuple(data),
    )


def _read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
    raw = path.read_bytes()
    index = 0

    def token() -> bytes:
        nonlocal index
        while index < len(raw):
            if raw[index:index + 1] == b"#":
                newline = raw.find(b"\n", index)
                index = len(raw) if newline < 0 else newline + 1
            elif raw[index:index + 1].isspace():
                index += 1
            else:
                break
        start = index
        while index < len(raw) and not raw[index:index + 1].isspace():
            index += 1
        if start == index:
            raise ValueError(f"invalid PGM header: {path}")
        return raw[start:index]

    magic = token()
    width = int(token())
    height = int(token())
    max_value = int(token())
    if width <= 0 or height <= 0 or not (0 < max_value <= 65535):
        raise ValueError(f"invalid PGM dimensions or range: {path}")
    count = width * height
    if magic == b"P2":
        pixels = [int(token()) for _ in range(count)]
    elif magic == b"P5":
        if index >= len(raw) or not raw[index:index + 1].isspace():
            raise ValueError(f"invalid PGM raster separator: {path}")
        index += 1
        bytes_per_pixel = 1 if max_value < 256 else 2
        raster = raw[index:index + count * bytes_per_pixel]
        if len(raster) != count * bytes_per_pixel:
            raise ValueError(f"truncated PGM raster: {path}")
        if bytes_per_pixel == 1:
            pixels = list(raster)
        else:
            pixels = [
                int.from_bytes(raster[offset:offset + 2], "big")
                for offset in range(0, len(raster), 2)
            ]
    else:
        raise ValueError(f"only P2/P5 PGM maps are supported: {path}")
    if any(pixel < 0 or pixel > max_value for pixel in pixels):
        raise ValueError(f"PGM pixel is outside declared range: {path}")
    return width, height, max_value, pixels


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
