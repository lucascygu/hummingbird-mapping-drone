# Generated / assisted by Grok
import math
import random
import struct

import pytest

from hummingbird_px4_bridge import frames as f

F = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
M_ENU_NED = ((0, 1, 0), (1, 0, 0), (0, 0, -1))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def assert_mat_close(a, b, tol=1e-9):
    for i in range(3):
        for j in range(3):
            assert a[i][j] == pytest.approx(b[i][j], abs=tol)


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rand_q(rng):
    return f.q_normalize(tuple(rng.gauss(0, 1) for _ in range(4)))


def test_identity_maps_to_identity():
    p, q = f.flu_world_to_frd((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    assert p == (0.0, -0.0, -0.0)
    assert q == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_flu_position_to_frd():
    p, _ = f.flu_world_to_frd((1.0, 2.0, 3.0), (1.0, 0.0, 0.0, 0.0))
    assert p == (1.0, -2.0, -3.0)


def test_left_turn_is_negative_yaw_in_frd():
    q_flu = f.q_from_rpy(0.0, 0.0, math.radians(90))  # CCW seen from above = left turn
    _, q = f.flu_world_to_frd((0, 0, 0), q_flu)
    assert math.degrees(yaw_of(q)) == pytest.approx(-90.0)


def test_nose_up_pitch_keeps_sign_convention():
    # Nose up is negative pitch about FLU y (left) and positive pitch about FRD y (right).
    q_flu = f.q_from_rpy(0.0, math.radians(-10), 0.0)
    _, q = f.flu_world_to_frd((0, 0, 0), q_flu)
    w, x, y, z = q
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    assert math.degrees(pitch) == pytest.approx(10.0)


@pytest.mark.parametrize("seed", range(20))
def test_frd_matches_matrix_definition(seed):
    rng = random.Random(seed)
    q = rand_q(rng)
    _, q_out = f.flu_world_to_frd((0, 0, 0), q)
    assert_mat_close(f.q_to_matrix(q_out), matmul(matmul(F, f.q_to_matrix(q)), F))


@pytest.mark.parametrize("seed", range(20))
def test_ned_matches_matrix_definition(seed):
    rng = random.Random(100 + seed)
    q = rand_q(rng)
    p = tuple(rng.uniform(-50, 50) for _ in range(3))
    p_out, q_out = f.enu_world_to_ned(p, q)
    assert p_out == pytest.approx((p[1], p[0], -p[2]))
    assert_mat_close(f.q_to_matrix(q_out), matmul(matmul(M_ENU_NED, f.q_to_matrix(q)), F))


def test_enu_facing_east_is_ned_yaw_90():
    _, q = f.enu_world_to_ned((0, 0, 0), (1.0, 0.0, 0.0, 0.0))
    assert math.degrees(yaw_of(q)) == pytest.approx(90.0)


def test_enu_facing_north_is_ned_yaw_0():
    _, q = f.enu_world_to_ned((0, 0, 0), f.q_from_rpy(0, 0, math.radians(90)))
    assert math.degrees(yaw_of(q)) == pytest.approx(0.0, abs=1e-9)


def test_compose_invert_roundtrip():
    rng = random.Random(7)
    p, q = (1.0, -2.0, 0.5), rand_q(rng)
    pi, qi = f.invert(p, q)
    p0, q0 = f.compose(p, q, pi, qi)
    assert p0 == pytest.approx((0, 0, 0), abs=1e-12)
    assert f.q_normalize(q0) == pytest.approx((1, 0, 0, 0), abs=1e-12)


def test_upside_down_mount_reexpressed():
    q_imu_base = f.q_from_rpy(math.pi, 0, 0)  # Mid-360 mounted upside-down
    p_imu_base = (0.0, 0.0, 0.0)
    # Vehicle climbs 1 m: the inverted IMU's z axis points down, so FAST-LIO sees z = -1.
    p, q = f.reexpress_in_vehicle_frame((0.0, 0.0, -1.0), (1, 0, 0, 0), p_imu_base, q_imu_base)
    assert p == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)
    assert f.q_normalize(q) == pytest.approx((1, 0, 0, 0), abs=1e-12)
    # Vehicle moves forward 1 m (x of inverted IMU still points forward).
    p, _ = f.reexpress_in_vehicle_frame((1.0, 0.0, 0.0), (1, 0, 0, 0), p_imu_base, q_imu_base)
    assert p == pytest.approx((1.0, 0.0, 0.0), abs=1e-12)


def test_lever_arm_rotation_about_imu():
    # base_link is 0.1 m in front of the IMU; yaw 90 deg about the IMU moves base_link sideways.
    p, _ = f.reexpress_in_vehicle_frame(
        (0, 0, 0), f.q_from_rpy(0, 0, math.pi / 2), (0.1, 0, 0), (1, 0, 0, 0)
    )
    # Relative to base_link at start: base moved from (0,0) to (-0.1, 0.1) in the start frame
    assert p == pytest.approx((-0.1, 0.1, 0.0), abs=1e-12)


def test_float32_quaternion_passes_px4_norm_check():
    rng = random.Random(3)
    for _ in range(200):
        _, q = f.flu_world_to_frd((0, 0, 0), rand_q(rng))
        q32 = struct.unpack("4f", struct.pack("4f", *q))
        assert abs(1.0 - math.sqrt(sum(c * c for c in q32))) <= 1e-5  # EKF2.cpp eps


def test_fastlio_covariance_index_swap():
    cov = [0.0] * 36
    cov[0], cov[7], cov[14] = 0.1, 0.2, 0.3   # rotation (FAST-LIO layout)
    cov[21], cov[28], cov[35] = 1.0, 2.0, 3.0  # position (FAST-LIO layout)
    pv, ov = f.diag_from_fastlio_covariance(cov)
    assert pv == (1.0, 2.0, 3.0) and ov == (0.1, 0.2, 0.3)
    pv, ov = f.diag_from_ros_covariance(cov)
    assert pv == (0.1, 0.2, 0.3) and ov == (1.0, 2.0, 3.0)


def test_normalize_rejects_zero():
    with pytest.raises(ValueError):
        f.q_normalize((0.0, 0.0, 0.0, 0.0))


# --------------------------------------------------------------------------- added by Claude
def test_rotate_variance_identity_and_quarter_turns():
    var = (1.0, 4.0, 9.0)
    assert f.rotate_variance(var, (1.0, 0.0, 0.0, 0.0)) == pytest.approx(var)
    assert f.rotate_variance(var, f.q_from_rpy(0.0, 0.0, math.pi / 2)) == pytest.approx((4.0, 1.0, 9.0))
    assert f.rotate_variance(var, f.q_from_rpy(math.pi / 2, 0.0, 0.0)) == pytest.approx((1.0, 9.0, 4.0))
    assert f.rotate_variance(var, f.q_from_rpy(math.pi, 0.0, 0.0)) == pytest.approx(var)


@pytest.mark.parametrize("seed", range(10))
def test_rotate_variance_matches_matrix_definition(seed):
    rng = random.Random(seed)
    q = rand_q(rng)
    var = tuple(rng.uniform(0.01, 2.0) for _ in range(3))
    r = f.q_to_matrix(q)
    d = ((var[0], 0, 0), (0, var[1], 0), (0, 0, var[2]))
    rt = tuple(tuple(r[j][i] for j in range(3)) for i in range(3))
    full = matmul(matmul(r, d), rt)
    out = f.rotate_variance(var, q)
    assert out == pytest.approx((full[0][0], full[1][1], full[2][2]))
    assert sum(out) == pytest.approx(sum(var))        # a rotation preserves the trace
    assert min(out) >= min(var) - 1e-12 and max(out) <= max(var) + 1e-12
