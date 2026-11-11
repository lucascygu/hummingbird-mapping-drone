# Generated / assisted by Claude
"""The synthetic flight must be self-consistent, otherwise it cannot be used as ground truth."""

import math

import numpy as np
import pytest

from hummingbird_bringup import sim_world as sw
from hummingbird_bringup.sim_check import score_trajectory, yaw_of_xyzw


def test_flight_starts_and_ends_at_rest_at_the_same_place():
    f = sw.Flight()
    p0, r0 = f.relative_pose(0.0)
    np.testing.assert_allclose(p0, 0.0, atol=1e-12)
    np.testing.assert_allclose(r0, np.eye(3), atol=1e-12)
    for t in (0.5, f.t_rest - 0.1, f.duration + 1.0):
        np.testing.assert_allclose(f.position(t), f.start, atol=1e-9)
        acc, gyro = f.imu(t)
        np.testing.assert_allclose(acc, [0.0, 0.0, sw.G], atol=1e-6)      # only gravity, z up
        np.testing.assert_allclose(gyro, 0.0, atol=1e-9)
    mid = f.position(f.t_rest + f.t_climb + f.t_lap / 2)
    assert mid[0] == pytest.approx(2 * f.radii[0]) and mid[2] > 1.0       # far side of the lap, airborne


def test_flight_stays_clear_of_every_obstacle_and_inside_the_room():
    f = sw.Flight()
    p = f.position(np.linspace(0.0, f.duration, 4000))
    lo, hi = sw.ROOM
    assert np.all(p > lo + 0.25) and np.all(p < hi - 0.25)
    for bmin, bmax in sw.OBSTACLES:
        gap = np.maximum(np.maximum(bmin - p, p - bmax), 0.0)             # distance to the box per axis
        assert np.linalg.norm(gap, axis=1).min() > 1.0


def test_imu_integrates_back_to_the_trajectory():
    """Strapdown-integrate the synthetic IMU: it must reproduce the analytic pose."""
    f = sw.Flight(t_rest=1.0, t_climb=2.0, t_lap=8.0, t_land=2.0)
    dt = 0.002
    ts = np.arange(0.0, f.duration, dt)
    acc_b, gyro_b = f.imu(ts + dt / 2)                                    # midpoint samples
    r = f.rotation(0.0)
    v = np.zeros(3)
    p = f.position(0.0).copy()
    g = np.array([0.0, 0.0, -sw.G])
    for k in range(ts.shape[0]):
        w = gyro_b[k] * dt
        angle = np.linalg.norm(w)
        kx = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        if angle > 1e-12:
            d_r = np.eye(3) + math.sin(angle) / angle * kx + (1 - math.cos(angle)) / angle ** 2 * kx @ kx
        else:
            d_r = np.eye(3) + kx
        r_mid = r @ (np.eye(3) + 0.5 * kx)
        a_w = r_mid @ acc_b[k] + g
        p = p + v * dt + 0.5 * a_w * dt * dt
        v = v + a_w * dt
        r = r @ d_r
    np.testing.assert_allclose(p, f.position(f.duration), atol=0.02)
    np.testing.assert_allclose(v, 0.0, atol=0.01)
    np.testing.assert_allclose(r, f.rotation(f.duration), atol=2e-3)


def test_tilted_start_is_identity_in_fast_lio_frame_and_sees_tilted_gravity():
    f = sw.Flight(start_roll=0.1, start_pitch=-0.05)
    p0, r0 = f.relative_pose(0.0)
    np.testing.assert_allclose(r0, np.eye(3), atol=1e-12)
    acc, _ = f.imu(0.5)
    assert np.linalg.norm(acc) == pytest.approx(sw.G, abs=1e-6)
    assert abs(acc[0]) > 0.3 and abs(acc[1]) > 0.5                        # gravity leaks into x / y


def test_raycast_hits_walls_floor_and_obstacles():
    o = np.array([[0.0, 0.0, 1.0]] * 5)
    d = np.array([[1.0, 0.0, 0.0],      # +x: pillar front face at x = 2.5
                  [-1.0, 0.0, 0.0],     # -x: room wall at x = -8
                  [0.0, 0.0, -1.0],     # down: floor
                  [0.0, 0.0, 1.0],      # up: ceiling at z = 5
                  [0.0, 1.0, 0.0]])     # +y: wall at y = 6
    rng, ok = sw.raycast(o, d)
    assert ok.all()
    np.testing.assert_allclose(rng, [2.5, 8.0, 1.0, 4.0, 6.0], atol=1e-9)
    # above the pillar the same ray reaches the far wall instead
    rng, ok = sw.raycast(np.array([[0.0, 0.0, 3.5]]), np.array([[1.0, 0.0, 0.0]]))
    assert rng[0] == pytest.approx(12.0)
    # out of range
    _, ok = sw.raycast(o[:1], d[:1], max_range=1.0)
    assert not ok[0]


def test_scan_points_lie_on_room_surfaces():
    f = sw.Flight()
    sim = sw.ScanSimulator(flight=f, points_per_scan=4000, range_noise=0.0, seed=3)
    t0 = f.t_rest + f.t_climb + 7.0                                        # mid-flight, moving and yawing
    offset, xyz = sim.scan(t0)
    assert xyz.shape[0] > 3000 and np.all(np.diff(offset) > 0) and offset[-1] < sim.scan_period
    el = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
    assert el.min() >= sw.FOV_ELEVATION_DEG[0] - 1e-6 and el.max() <= sw.FOV_ELEVATION_DEG[1] + 1e-6
    # put every point back into the world with the pose it was measured from
    t = t0 + offset
    r = f.rotation(t)
    world = f.position(t) + np.einsum("nij,nj->ni", r, xyz + sw.IMU_T_LIDAR)
    lo, hi = sw.ROOM
    on_room = np.isclose(world, lo, atol=1e-6).any(axis=1) | np.isclose(world, hi, atol=1e-6).any(axis=1)
    on_box = np.zeros(world.shape[0], bool)
    for bmin, bmax in sw.OBSTACLES:
        inside = np.all((world >= bmin - 1e-6) & (world <= bmax + 1e-6), axis=1)
        face = np.isclose(world, bmin, atol=1e-6).any(axis=1) | np.isclose(world, bmax, atol=1e-6).any(axis=1)
        on_box |= inside & face
    assert (on_room | on_box).all()
    assert on_box.sum() > 50 and on_room.sum() > 2000                      # it sees both kinds of surface


def test_quaternion_from_matrix():
    for yaw, pitch, roll in [(0.0, 0.0, 0.0), (1.0, 0.2, -0.3), (3.0, -0.1, 0.05), (-2.5, 0.0, 3.0)]:
        r = sw.rot_zyx(yaw, pitch, roll)
        x, y, z, w = sw.quat_xyzw_from_matrix(r)
        assert x * x + y * y + z * z + w * w == pytest.approx(1.0)
        back = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        np.testing.assert_allclose(back, r, atol=1e-12)
    assert yaw_of_xyzw(*sw.quat_xyzw_from_matrix(sw.rot_zyx(0.8, 0.0, 0.0))) == pytest.approx(0.8)


def test_score_trajectory():
    t = np.arange(0.0, 20.0, 0.1)
    truth = [(ti, ti, 0.5 * ti, 1.0, 0.1 * ti) for ti in t]
    perfect = score_trajectory(truth[5:-5], truth)
    assert perfect["n"] == len(truth) - 10 and perfect["rmse_m"] == pytest.approx(0.0, abs=1e-9)
    off = [(ti + 0.05, ti + 0.05 + 0.3, 0.5 * (ti + 0.05), 1.0 + 0.4, 0.1 * (ti + 0.05) + 0.02) for ti in t[:-1]]
    s = score_trajectory(off, truth)                                       # stamps between truth samples
    assert s["rmse_m"] == pytest.approx(0.5, abs=1e-6) and s["max_m"] == pytest.approx(0.5, abs=1e-6)
    assert s["yaw_rmse_deg"] == pytest.approx(math.degrees(0.02), abs=1e-6)
    assert score_trajectory([], truth)["n"] == 0
    assert score_trajectory([(100.0, 0, 0, 0, 0), (101.0, 0, 0, 0, 0)], truth)["n"] == 0   # no overlap


def test_distance_to_surfaces():
    pts = np.array([[0.0, 0.0, 0.0],        # on the floor
                    [12.0, 1.0, 2.0],       # on the +x wall
                    [2.5, 0.0, 1.0],        # on the pillar's front face
                    [2.85, -0.05, 1.0],     # centre of the pillar: 0.35 m deep
                    [0.0, 0.0, 2.0],        # mid-air, 2 m above the floor
                    [1.25, 4.25, 4.0]])     # inside the column: 0.25 m from its faces
    np.testing.assert_allclose(sw.distance_to_surfaces(pts), [0.0, 0.0, 0.0, 0.35, 2.0, 0.25], atol=1e-9)
    # a noiseless scan scores (almost) zero
    f = sw.Flight()
    sim = sw.ScanSimulator(flight=f, points_per_scan=2000, range_noise=0.0)
    offset, xyz = sim.scan(10.0)
    world = f.position(10.0 + offset) + np.einsum("nij,nj->ni", f.rotation(10.0 + offset), xyz + sw.IMU_T_LIDAR)
    assert sw.distance_to_surfaces(world).max() < 1e-6
