"""Numerical perception, optional VO admission and visibility-aware motion checks."""

from dataclasses import dataclass
import math

import numpy as np


def fresh(stamp, now, timeout, future_tolerance=0.05):
    return (math.isfinite(stamp) and math.isfinite(now) and stamp > 0
            and -future_tolerance <= now - stamp <= timeout)


def rotation_matrix(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.01:
        raise ValueError('invalid rotation quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def decode_depth(data, width, height, step, encoding, bigendian=False):
    types = {'16UC1': 'u2', 'mono16': 'u2', '32FC1': 'f4'}
    if encoding not in types or width < 1 or height < 1:
        raise ValueError('unsupported depth encoding or dimensions')
    dtype = np.dtype(('>' if bigendian else '<') + types[encoding])
    if step < width * dtype.itemsize or len(data) != height * step:
        raise ValueError('depth buffer/stride mismatch')
    image = np.ndarray((height, width), dtype=dtype, buffer=bytes(data),
                       strides=(step, dtype.itemsize)).astype(float)
    return image * (0.001 if dtype.itemsize == 2 else 1.0)


def camera_intrinsics(k, p):
    """Use the rectified projection model, or a valid unskewed K fallback.

    Astra registered depth can publish NaNs in K while P correctly describes
    the registered image. Never invent focal lengths or principal points.
    """
    if len(p) == 12 and all(math.isfinite(x) for x in p):
        if (p[0] > 0 and p[5] > 0 and abs(p[10]-1) < 1e-8
                and all(abs(p[i]) < 1e-8 for i in (1, 3, 4, 7, 8, 9, 11))):
            return p[0], p[5], p[2], p[6]
    if len(k) == 9 and all(math.isfinite(x) for x in k):
        if (k[0] > 0 and k[4] > 0 and abs(k[8]-1) < 1e-8
                and all(abs(k[i]) < 1e-8 for i in (1, 3, 6, 7))):
            return k[0], k[4], k[2], k[5]
    raise ValueError('no valid rectified P or camera K intrinsics')


@dataclass(frozen=True)
class DepthConfig:
    range_min: float = 0.25
    range_max: float = 4.0
    obstacle_min_z: float = 0.05
    obstacle_max_z: float = 1.20
    stride: int = 4
    bins: int = 181
    min_valid_fraction: float = 0.60
    min_samples: int = 3

    def __post_init__(self):
        if not (0 < self.range_min < self.range_max and self.stride >= 1
                and self.bins >= 3 and self.obstacle_min_z < self.obstacle_max_z
                and 0 < self.min_valid_fraction <= 1 and self.min_samples >= 1):
            raise ValueError('invalid depth configuration')


def depth_scan(image, intrinsics, translation, quaternion, config=DepthConfig()):
    """Return obstacle ranges and conservative observed distances in base XY.

    Unknown angular bins remain NaN. Infinite obstacle range only means that
    enough valid samples in this bin contained no obstacle in the height band;
    clearance separately limits how far the camera actually observed.
    """
    fx, fy, cx, cy = map(float, intrinsics)
    if not all(map(math.isfinite, (fx, fy, cx, cy))) or fx <= 0 or fy <= 0:
        raise ValueError('invalid camera intrinsics')
    translation = np.asarray(translation, dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('invalid camera translation')
    rotation = rotation_matrix(quaternion)
    h, w = image.shape
    rows, cols = np.mgrid[0:h:config.stride, 0:w:config.stride]
    z = image[::config.stride, ::config.stride].ravel()
    rays = np.column_stack(((cols.ravel()-cx)/fx, (rows.ravel()-cy)/fy,
                            np.ones(z.size))) @ rotation.T
    step = 2*math.pi/(config.bins-1)
    valid = np.isfinite(z) & (z >= config.range_min) & (z <= config.range_max)
    points = rays[valid] * z[valid, None] + translation
    distances = np.hypot(points[:, 0], points[:, 1])
    angles = np.arctan2(points[:, 1], points[:, 0])
    indices = np.clip(np.rint((angles+math.pi)/step).astype(int), 0, config.bins-1)
    counts = np.bincount(indices, minlength=config.bins)
    # For missing pixels conservatively mark every possible angular bin between
    # the near and far projection, accounting for camera translation/parallax.
    missing_rays = rays[~valid]
    near = missing_rays * config.range_min + translation
    far = missing_rays * config.range_max + translation
    near_indices = np.clip(np.rint((np.arctan2(near[:, 1], near[:, 0])+math.pi)/step)
                           .astype(int), 0, config.bins-1)
    far_indices = np.clip(np.rint((np.arctan2(far[:, 1], far[:, 0])+math.pi)/step)
                          .astype(int), 0, config.bins-1)
    missing = np.zeros(config.bins+1, dtype=int)
    np.add.at(missing, np.minimum(near_indices, far_indices), 1)
    np.add.at(missing, np.maximum(near_indices, far_indices)+1, -1)
    expected_total = counts + np.cumsum(missing[:-1])
    fraction = counts / expected_total.clip(min=1)
    covered = (counts >= config.min_samples) & (fraction >= config.min_valid_fraction)
    clearance = np.full(config.bins, np.inf)
    np.minimum.at(clearance, indices, distances)
    # Even invalid height returns constrain visibility, rather than becoming
    # unlimited free space. Floor points constrain visibility to the floor hit.
    clearance = np.minimum(clearance, config.range_max)
    obstacles = np.full(config.bins, np.inf)
    mask = ((points[:, 2] >= config.obstacle_min_z)
            & (points[:, 2] <= config.obstacle_max_z))
    np.minimum.at(obstacles, indices[mask], distances[mask])
    # A real obstacle remains an obstacle even when surrounding depth is sparse.
    obstacles[(~covered) & (~np.isfinite(obstacles))] = np.nan
    clearance[~covered] = np.nan
    return obstacles, clearance, float(valid.mean()) if z.size else 0.0


class VOGate:
    """VO never owns the odometry origin; only vetted body-frame vx/vy pass."""

    def __init__(self, recovery_samples=5, timeout=0.4, min_inliers=30,
                 min_ratio=0.4, max_speed=0.6, max_yaw_rate=1.0,
                 max_pose_step=0.25, variance_floor=0.04, variance_max=1.0):
        self.recovery_samples = recovery_samples
        self.timeout = timeout
        self.min_inliers = min_inliers
        self.min_ratio = min_ratio
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.max_pose_step = max_pose_step
        self.variance_floor = variance_floor
        self.variance_max = variance_max
        self.state = 'INITIALIZING'
        self.good = 0
        self.previous = None
        self.reason = 'waiting for VO'

    def reject(self, reason):
        self.state, self.good, self.previous = 'LOST', 0, None
        self.reason = reason
        return None

    def admit(self, *, stamp, now, velocity, covariance, position, yaw,
              lost, inliers, matches, interval, distance_travelled):
        if not fresh(stamp, now, self.timeout):
            return self.reject('stale/future VO')
        if lost or inliers < self.min_inliers or matches <= 0 or inliers/matches < self.min_ratio:
            return self.reject('tracking quality')
        values = (*velocity, *position, yaw, interval, distance_travelled)
        if not all(map(math.isfinite, values)) or not 0 < interval <= self.timeout:
            return self.reject('invalid VO values/interval')
        if math.hypot(*velocity[:2]) > self.max_speed or abs(velocity[2]) > self.max_yaw_rate:
            return self.reject('implausible VO speed')
        covariance = np.asarray(covariance, dtype=float).reshape(6, 6)
        planar = covariance[np.ix_([0, 1], [0, 1])]
        if (not np.isfinite(covariance).all() or not np.allclose(planar, planar.T)
                or np.linalg.eigvalsh(planar).min() <= 0
                or np.diag(planar).max() > self.variance_max):
            return self.reject('invalid VO covariance')
        if self.previous is not None:
            last_stamp, last_position, last_yaw, last_distance = self.previous
            dt = stamp-last_stamp
            if dt <= 0 or dt > self.timeout or distance_travelled < last_distance-0.001:
                return self.reject('timestamp gap/order or VO reset')
            delta = math.dist(position, last_position)
            angle = abs(math.atan2(math.sin(yaw-last_yaw), math.cos(yaw-last_yaw)))
            if delta > min(self.max_pose_step, self.max_speed*dt+0.03) or angle > self.max_yaw_rate*dt+0.05:
                return self.reject('pose discontinuity/reset')
        self.previous = (stamp, position, yaw, distance_travelled)
        self.good += 1
        self.state = 'VALID' if self.good >= self.recovery_samples else 'RECOVERING'
        self.reason = 'accepted' if self.state == 'VALID' else 'confirming tracking'
        if self.state != 'VALID':
            return None
        # Preserve correlations and only increase uncertainty (PSD addition).
        planar += np.eye(2)*max(0.0, self.variance_floor-np.linalg.eigvalsh(planar).min())
        output = np.eye(6)*1e6
        output[np.ix_([0, 1], [0, 1])] = planar
        return output.ravel().tolist()


@dataclass(frozen=True)
class MotionConfig:
    front: float = 0.0
    rear: float = 0.0
    half_width: float = 0.0
    margin: float = 0.03
    max_linear: float = 0.35
    max_angular: float = 0.40
    reaction_time: float = 0.30
    deceleration: float = 0.35
    calibrated: bool = False


def supervise_motion(linear, angular, measured_linear, measured_angular,
                     obstacles, clearance, angle_min, angle_increment,
                     range_min, config=MotionConfig()):
    """Check a stopping envelope, including newly swept body boundary points.

    The existing body interior is occupied by the robot; newly swept space
    must be observed. No memory or assumption of free rear space is invented.
    """
    values = (linear, angular, measured_linear, measured_angular, angle_min,
              angle_increment, range_min)
    if not all(map(math.isfinite, values)):
        return 0.0, 0.0, 'non-finite command or measurement'
    if linear == 0 and angular == 0:
        return 0.0, 0.0, 'stopped'
    if not config.calibrated or min(config.front, config.rear, config.half_width) <= 0:
        return 0.0, 0.0, 'camera/footprint calibration required'
    if linear < 0:
        return 0.0, 0.0, 'reverse disabled: rear is unobserved'
    if (angle_increment <= 0 or range_min <= 0 or len(obstacles) != len(clearance)
            or len(obstacles) < 3):
        return 0.0, 0.0, 'invalid depth scan geometry'
    v = min(linear, config.max_linear)
    w = max(-config.max_angular, min(config.max_angular, angular))
    front = config.front+config.margin
    rear = config.rear+config.margin
    half = config.half_width+config.margin
    radius = math.hypot(max(front, rear), half)
    # Include residual measured motion while braking, even for a turn command.
    speed = max(abs(v), abs(measured_linear))
    spin = max(abs(w), abs(measured_angular))
    stop_time = max(speed/config.deceleration, radius*spin/config.deceleration)
    horizon = config.reaction_time + stop_time
    # Avoid stepping over thin objects; use <= 1 cm displacement at boundary.
    dt = min(0.05, 0.01/max(speed+radius*spin, 0.01))
    boundary = []
    for x in np.linspace(-rear, front, max(3, math.ceil((front+rear)/0.02))):
        boundary.extend(((x, -half), (x, half)))
    for y in np.linspace(-half, half, max(3, math.ceil(2*half/0.02))):
        boundary.extend(((-rear, y), (front, y)))
    angles = angle_min + np.arange(len(obstacles))*angle_increment
    obs = [(r*math.cos(a), r*math.sin(a)) for r, a in zip(obstacles, angles)
           if math.isfinite(r) and r > 0]
    # Check both commanded trajectory and braking continuation of measured motion.
    for path_v, path_w in ((v, w), (measured_linear, measured_angular)):
        x = y = yaw = 0.0
        for _ in range(max(1, math.ceil(horizon/dt))):
            x += path_v*math.cos(yaw)*dt
            y += path_v*math.sin(yaw)*dt
            yaw += path_w*dt
            c, s = math.cos(yaw), math.sin(yaw)
            for ox, oy in obs:
                bx, by = c*(ox-x)+s*(oy-y), -s*(ox-x)+c*(oy-y)
                if -rear <= bx <= front and abs(by) <= half:
                    return 0.0, 0.0, 'obstacle in stopping envelope'
            for bx, by in boundary:
                px, py = x+c*bx-s*by, y+s*bx+c*by
                if -rear <= px <= front and abs(py) <= half:
                    continue
                r = math.hypot(px, py)
                idx = round((math.atan2(py, px)-angle_min)/angle_increment)
                if not (0 <= idx < len(clearance)) or r < range_min:
                    return 0.0, 0.0, 'unobserved swept space / near blind zone'
                observed = clearance[idx]
                if not math.isfinite(observed) or observed < r:
                    return 0.0, 0.0, 'unknown/insufficient observed stopping distance'
    return v, w, 'clear'
