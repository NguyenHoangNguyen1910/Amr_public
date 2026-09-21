"""Stateless geometry and sensor-boundary helpers."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence

from .config import NavigationConfig
from .types import NavigationInput, Point2D, Pose2D, Velocity


def distance(first: Point2D, second: Point2D) -> float:
    return math.hypot(second.x - first.x, second.y - first.y)


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def lerp(start: Point2D, end: Point2D, amount: float) -> Point2D:
    return Point2D(
        start.x + (end.x - start.x) * amount,
        start.y + (end.y - start.y) * amount,
    )


def transform_point(pose: Pose2D, point: Point2D) -> Point2D:
    cosine, sine = math.cos(pose.yaw), math.sin(pose.yaw)
    return Point2D(
        pose.x + point.x * cosine - point.y * sine,
        pose.y + point.x * sine + point.y * cosine,
    )


def transform_points(pose: Pose2D, points: Iterable[Point2D]) -> tuple[Point2D, ...]:
    return tuple(transform_point(pose, point) for point in points)


def distance_to_segment(point: Point2D, start: Point2D, end: Point2D) -> float:
    dx, dy = end.x - start.x, end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return distance(point, start)
    amount = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
    return distance(point, lerp(start, end, min(1.0, max(0.0, amount))))


def path_length(path: Sequence[Point2D]) -> float:
    return sum(distance(first, second) for first, second in zip(path, path[1:]))


def resample_path(path: Sequence[Point2D], spacing: float) -> tuple[Point2D, ...]:
    if spacing <= 0.0:
        raise ValueError("path spacing must be positive")
    if len(path) < 2:
        return tuple(path)
    result = [path[0]]
    for start, end in zip(path, path[1:]):
        divisions = max(1, math.ceil(distance(start, end) / spacing))
        result.extend(lerp(start, end, index / divisions) for index in range(1, divisions + 1))
    result[-1] = path[-1]
    return tuple(result)


def nearest_index(point: Point2D, path: Sequence[Point2D]) -> int:
    if not path:
        raise ValueError("path is empty")
    return min(range(len(path)), key=lambda index: distance(point, path[index]))


def target_point(point: Point2D, path: Sequence[Point2D], lookahead: float) -> Point2D:
    index = nearest_index(point, path)
    travelled = distance(point, path[index])
    for current in range(index, len(path) - 1):
        travelled += distance(path[current], path[current + 1])
        if travelled >= lookahead:
            return path[current + 1]
    return path[-1]


def min_distance(point: Point2D, others: Iterable[Point2D]) -> float:
    return min((distance(point, other) for other in others), default=math.inf)


# -- sensor boundary ----------------------------------------------------------


class SensorDataError(RuntimeError):
    pass


def _mapping(value: object, message: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise SensorDataError(message)
    return value


def _finite(value: object, message: str) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise SensorDataError(message) from error
    if not math.isfinite(number):
        raise SensorDataError(message)
    return number


def parse_lidar_points(
    lidar: Mapping[str, object], config: NavigationConfig
) -> tuple[Point2D, ...]:
    """Convert a validated LaserScan snapshot to robot-frame points."""

    required = ("angle_min", "angle_step", "range_min", "range_max", "count", "ranges")
    if any(lidar.get(name) is None for name in required):
        raise SensorDataError("LiDAR metadata is incomplete")
    try:
        angle_min = float(lidar["angle_min"])
        angle_step = float(lidar["angle_step"])
        range_min = float(lidar["range_min"])
        range_max = float(lidar["range_max"])
        count = int(lidar["count"])
        invalid_count = int(lidar.get("invalid_count", 0))
    except (TypeError, ValueError) as error:
        raise SensorDataError("LiDAR metadata is invalid") from error
    ranges = lidar["ranges"]
    if not isinstance(ranges, Sequence) or isinstance(ranges, (str, bytes)):
        raise SensorDataError("LiDAR ranges are invalid")
    if config.lidar_count and count != config.lidar_count:
        raise SensorDataError(f"LiDAR scan must contain {config.lidar_count} rays")
    if len(ranges) != count:
        raise SensorDataError("LiDAR range count does not match metadata")
    if invalid_count:
        raise SensorDataError("LiDAR scan contains NaN values")
    if not all(math.isfinite(v) for v in (angle_min, angle_step, range_min, range_max)):
        raise SensorDataError("LiDAR metadata must be finite")
    if angle_step <= 0.0 or range_min <= 0.0 or range_max <= range_min:
        raise SensorDataError("LiDAR metadata is invalid")

    points = []
    for index, raw_range in enumerate(ranges):
        if raw_range is None:
            continue
        measured = _finite(raw_range, "LiDAR ranges must be finite or null")
        measured = min(range_max, max(range_min, measured))
        if measured >= range_max - 1e-6:
            continue  # nothing within range along this ray
        angle = angle_min + index * angle_step + config.lidar_sensor_yaw_rad
        points.append(
            Point2D(
                config.lidar_sensor_x_m + measured * math.cos(angle),
                config.lidar_sensor_y_m + measured * math.sin(angle),
            )
        )
    return tuple(points)


def parse_navigation_input(
    snapshot: Mapping[str, object], config: NavigationConfig
) -> NavigationInput:
    """Validate one transport snapshot and return its typed navigation view."""

    freshness = _mapping(snapshot.get("freshness"), "sensor freshness is unavailable")
    for name in ("lidar", "odometry", "model_pose"):
        item = _mapping(freshness.get(name), f"{name} has not been received")
        age = item.get("age_seconds")
        if not item.get("received") or age is None:
            raise SensorDataError(f"{name} has not been received")
        try:
            stale = float(age) > config.sensor_timeout_seconds  # type: ignore[arg-type]
        except (TypeError, ValueError) as error:
            raise SensorDataError(f"{name} timestamp is invalid") from error
        if stale:
            raise SensorDataError(f"{name} is stale")

    sensors = _mapping(snapshot.get("sensors"), "navigation sensors are unavailable")
    odometry = _mapping(sensors.get("odometry"), "odometry is unavailable")
    model_pose = _mapping(sensors.get("model_pose"), "model pose is unavailable")
    lidar = _mapping(sensors.get("lidar"), "LiDAR is unavailable")

    invalid = "model pose or odometry is invalid"
    try:
        position = _mapping(model_pose["position"], "model pose is invalid")
        pose = Pose2D(
            _finite(position["x"], invalid),
            _finite(position["y"], invalid),
            _finite(model_pose["yaw"], invalid),
        )
        model_linear = model_pose.get("linear_velocity")
        model_angular = model_pose.get("angular_velocity")
        if isinstance(model_linear, Mapping) and isinstance(model_angular, Mapping):
            velocity = Velocity(
                _finite(model_linear["x"], invalid),
                _finite(model_angular["z"], invalid),
            )
        else:
            velocity = Velocity(
                _finite(_mapping(odometry["linear_velocity"], "odometry is invalid")["x"], invalid),
                _finite(_mapping(odometry["angular_velocity"], "odometry is invalid")["z"], invalid),
            )
    except KeyError as error:
        raise SensorDataError(invalid) from error

    try:
        sequence = int(lidar.get("sequence", 0))  # type: ignore[arg-type]
    except (TypeError, ValueError) as error:
        raise SensorDataError("LiDAR sequence is invalid") from error
    if sequence < 0:
        raise SensorDataError("LiDAR sequence is invalid")

    return NavigationInput(pose, velocity, sequence, parse_lidar_points(lidar, config))
