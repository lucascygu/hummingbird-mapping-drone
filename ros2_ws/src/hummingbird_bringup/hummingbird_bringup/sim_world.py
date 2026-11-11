# Generated / assisted by Claude
"""Synthetic flight + room for exercising the mapping stack without hardware (numpy only).

Used by mid360_sim (publishes fake Livox Mid-360 data) and by the unit tests. Nothing here is a
physics simulation of the airframe: it is a smooth, known trajectory through a known room, which
is exactly what is needed to check FAST-LIO, the PX4 bridge and the map tools against ground truth.

Frames
  world  z up, gravity along -z, origin on the floor.
  imu    the Mid-360 built-in IMU (FAST-LIO "body"); FLU when the sensor is mounted level.
  lidar  same orientation as imu, origin offset by IMU_T_LIDAR (FAST-LIO extrinsic_T).
FAST-LIO's own world frame ("camera_init") is the imu frame at start-up, so the pose it should
report is ``relative_pose(t) = T_world_imu(0)^-1 * T_world_imu(t)``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

G = 9.81
# LiDAR origin in the IMU frame (FAST_LIO_ROS2 config/mid360.yaml extrinsic_T).
IMU_T_LIDAR = np.array([-0.011, -0.02329, 0.04412])
# Livox Mid-360 field of view: 360 deg horizontal, -7 .. +52 deg vertical.
FOV_ELEVATION_DEG = (-7.0, 52.0)

# Room (inside of this box) and obstacles (outside of these boxes), metres, (min xyz, max xyz).
ROOM = (np.array([-8.0, -6.0, 0.0]), np.array([12.0, 6.0, 5.0]))
OBSTACLES = (
    (np.array([2.5, -0.4, 0.0]), np.array([3.2, 0.3, 3.0])),      # pillar inside the flown lap
    (np.array([-4.0, 2.0, 0.0]), np.array([-2.5, 3.5, 1.2])),     # crate
    (np.array([7.0, 3.0, 0.0]), np.array([9.0, 5.0, 2.0])),       # cabinet
    (np.array([1.0, 4.0, 0.0]), np.array([1.5, 4.5, 5.0])),       # floor-to-ceiling column
    (np.array([-6.0, -4.0, 0.0]), np.array([-5.0, -1.5, 2.5])),   # shelf
)


def smoothstep(u):
    """Quintic 0 -> 1 with zero velocity and acceleration at both ends."""
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (u * (6.0 * u - 15.0) + 10.0)


def rot_zyx(yaw, pitch, roll):
    """Batch rotation matrices R = Rz(yaw) Ry(pitch) Rx(roll); inputs broadcast, output (..., 3, 3)."""
    yaw, pitch, roll = np.broadcast_arrays(np.asarray(yaw, float), np.asarray(pitch, float),
                                           np.asarray(roll, float))
    cy, sy, cp, sp, cr, sr = np.cos(yaw), np.sin(yaw), np.cos(pitch), np.sin(pitch), np.cos(roll), np.sin(roll)
    r = np.empty(yaw.shape + (3, 3))
    r[..., 0, 0] = cy * cp
    r[..., 0, 1] = cy * sp * sr - sy * cr
    r[..., 0, 2] = cy * sp * cr + sy * sr
    r[..., 1, 0] = sy * cp
    r[..., 1, 1] = sy * sp * sr + cy * cr
    r[..., 1, 2] = sy * sp * cr - cy * sr
    r[..., 2, 0] = -sp
    r[..., 2, 1] = cp * sr
    r[..., 2, 2] = cp * cr
    return r


def quat_xyzw_from_matrix(r) -> Tuple[float, float, float, float]:
    """ROS-order quaternion of one rotation matrix."""
    t = np.trace(r)
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2.0
        q = [0.0, 0.0, 0.0]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
        w = (r[k, j] - r[j, k]) / s
        x, y, z = q
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if w < 0:
        n = -n
    return (x / n, y / n, z / n, w / n)


@dataclass(frozen=True)
class Flight:
    """Stand still, climb, fly one lap of an ellipse while yawing a full turn, land, stand still."""

    start: Tuple[float, float, float] = (0.0, 0.0, 0.3)   # IMU position at rest, world frame
    start_yaw: float = 0.0
    start_roll: float = 0.0     # non-zero = the vehicle starts on a slope (tests gravity handling)
    start_pitch: float = 0.0
    t_rest: float = 3.0         # FAST-LIO needs a stationary start to initialise its IMU
    t_climb: float = 3.0
    t_lap: float = 30.0
    t_land: float = 3.0
    cruise_height: float = 1.2  # climb above the start height
    radii: Tuple[float, float] = (3.0, 2.0)
    tilt_rad: float = 0.06      # roll / pitch wobble during the lap
    loop: bool = False

    @property
    def duration(self) -> float:
        return self.t_rest + self.t_climb + self.t_lap + self.t_land

    def _phase(self, t):
        t = np.asarray(t, dtype=float)
        if self.loop:
            cycle = self.duration + self.t_rest
            t = np.mod(t, cycle)
        climb = smoothstep((t - self.t_rest) / self.t_climb)
        lap = smoothstep((t - self.t_rest - self.t_climb) / self.t_lap)
        land = smoothstep((t - self.t_rest - self.t_climb - self.t_lap) / self.t_land)
        return climb, lap, land

    def position(self, t):
        """IMU position in the world frame, shape (..., 3)."""
        climb, lap, land = self._phase(t)
        th = 2.0 * np.pi * lap
        rx, ry = self.radii
        p = np.empty(np.shape(th) + (3,))
        # Ellipse through the start point; starts tangentially along +y with zero speed.
        p[..., 0] = self.start[0] + rx * (1.0 - np.cos(th))
        p[..., 1] = self.start[1] + ry * np.sin(th)
        p[..., 2] = self.start[2] + self.cruise_height * (climb - land) + 0.25 * np.sin(2.0 * th)
        return p

    def rpy(self, t):
        climb, lap, land = self._phase(t)
        th = 2.0 * np.pi * lap
        active = climb * (1.0 - land)
        yaw = self.start_yaw + th
        roll = self.start_roll + self.tilt_rad * np.sin(3.0 * th) * active
        pitch = self.start_pitch + self.tilt_rad * np.sin(2.0 * th + 0.7) * active
        return roll, pitch, yaw

    def rotation(self, t):
        """R_world_imu, shape (..., 3, 3)."""
        roll, pitch, yaw = self.rpy(t)
        return rot_zyx(yaw, pitch, roll)

    def relative_pose(self, t):
        """(position, R) of the IMU at time t expressed in the IMU frame at t = 0 (FAST-LIO's world)."""
        r0 = self.rotation(0.0)
        p0 = self.position(0.0)
        p = np.einsum("ji,...j->...i", r0, self.position(t) - p0)
        r = np.einsum("ji,...jk->...ik", r0, self.rotation(t))
        return p, r

    def imu(self, t, h: float = 1e-3):
        """Ideal IMU readings at times t: (specific force in m/s^2, angular rate in rad/s), body frame."""
        t = np.asarray(t, dtype=float)
        acc_w = (self.position(t + h) - 2.0 * self.position(t) + self.position(t - h)) / (h * h)
        r = self.rotation(t)
        f_w = acc_w + np.array([0.0, 0.0, G])                       # a - g, with g = (0, 0, -G)
        f_b = np.einsum("...ji,...j->...i", r, f_w)
        # R(t-h)^T R(t+h) = exp([w]x 2h) for small h
        d = np.einsum("...ji,...jk->...ik", self.rotation(t - h), self.rotation(t + h))
        w_b = np.stack([d[..., 2, 1] - d[..., 1, 2], d[..., 0, 2] - d[..., 2, 0],
                        d[..., 1, 0] - d[..., 0, 1]], axis=-1) / (4.0 * h)
        return f_b, w_b


def raycast(origins: np.ndarray, directions: np.ndarray, max_range: float = 40.0):
    """Range along each ray to the first surface (room walls from inside, obstacles from outside).

    origins, directions: (N, 3), directions unit length. Returns (ranges, hit) with hit False where
    nothing is within max_range.
    """
    o = np.asarray(origins, float)
    d = np.asarray(directions, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        lo, hi = ROOM
        # Leaving the room: per axis the wall in the direction of travel.
        t_wall = np.where(d > 0, (hi - o) * inv, np.where(d < 0, (lo - o) * inv, np.inf))
        best = np.min(t_wall, axis=1)
        for bmin, bmax in OBSTACLES:
            t1 = (bmin - o) * inv
            t2 = (bmax - o) * inv
            # A ray parallel to a slab never enters it unless the origin already lies inside.
            inside = (o >= bmin) & (o <= bmax)
            parallel = d == 0
            tn = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2))
            tf = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2))
            t_near = np.max(tn, axis=1)
            t_far = np.min(tf, axis=1)
            hit = (t_near < t_far) & (t_near > 0.0)
            best = np.where(hit & (t_near < best), t_near, best)
    ok = np.isfinite(best) & (best > 0.0) & (best <= max_range)
    return np.where(ok, best, 0.0), ok


def distance_to_surfaces(points_world: np.ndarray) -> np.ndarray:
    """Distance of each world-frame point to the nearest wall, floor, ceiling or obstacle face.

    Used to score a saved map: a correct map only has points on those surfaces.
    """
    p = np.asarray(points_world, float).reshape(-1, 3)
    lo, hi = ROOM
    d = np.minimum(np.abs(p - lo), np.abs(p - hi)).min(axis=1)
    for bmin, bmax in OBSTACLES:
        outside = np.linalg.norm(np.maximum(np.maximum(bmin - p, p - bmax), 0.0), axis=1)
        inside = np.all((p > bmin) & (p < bmax), axis=1)
        depth = np.minimum(p - bmin, bmax - p).min(axis=1)
        d = np.minimum(d, np.where(inside, depth, outside))
    return d


@dataclass
class ScanSimulator:
    """One Mid-360-like frame: random directions in the sensor FOV, each ray cast from the pose
    the sensor has at that point's own time (so FAST-LIO's motion compensation has work to do)."""

    flight: Flight = field(default_factory=Flight)
    points_per_scan: int = 6000
    scan_period: float = 0.1
    range_noise: float = 0.02
    min_range: float = 0.1
    max_range: float = 40.0
    seed: int = 1

    def __post_init__(self):
        self._rng = np.random.default_rng(self.seed)

    def scan(self, t0: float):
        """Frame starting at sim time t0. Returns (offset_s, xyz in the lidar frame), both valid points only."""
        n = self.points_per_scan
        offset = (np.arange(n) + 0.5) * (self.scan_period / n)
        az = self._rng.uniform(0.0, 2.0 * np.pi, n)
        el = np.radians(self._rng.uniform(*FOV_ELEVATION_DEG, n))
        d_l = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
        t = t0 + offset
        r = self.flight.rotation(t)
        origin = self.flight.position(t) + np.einsum("nij,j->ni", r, IMU_T_LIDAR)
        d_w = np.einsum("nij,nj->ni", r, d_l)
        rng, ok = raycast(origin, d_w, self.max_range)
        rng = rng + self._rng.normal(0.0, self.range_noise, n)
        ok &= rng >= self.min_range
        return offset[ok], (d_l * rng[:, None])[ok]
