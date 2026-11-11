# Generated / assisted by Grok
"""Pure-python frame conversion helpers (no ROS, no numpy) so they can be unit-tested anywhere.

Conventions
-----------
* Quaternions are tuples ``(w, x, y, z)`` (Hamilton, right-handed), the same order PX4's
  ``VehicleOdometry.q`` uses. ROS messages store ``(x, y, z, w)``; convert at the edges.
* A quaternion ``q_AB`` rotates vectors expressed in frame B into frame A
  (i.e. it is the attitude of B relative to A). ``R(q_AB) @ v_B = v_A``.
* ROS (REP 103/105): world = ENU (or an FLU-like "odom" frame with arbitrary heading),
  body = FLU (x forward, y left, z up).
* PX4 (``px4_msgs/VehicleOdometry``): world = NED (``POSE_FRAME_NED``) or FRD with arbitrary
  heading (``POSE_FRAME_FRD``), body = FRD (x forward, y right, z down).

FAST-LIO2 publishes ``/Odometry`` with ``frame_id=camera_init`` (its IMU frame at start-up)
and ``child_frame_id=body`` (the IMU frame, i.e. the Mid-360 built-in IMU). Neither is
north-aligned, so the default output frame is ``POSE_FRAME_FRD``.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

Quat = Tuple[float, float, float, float]  # (w, x, y, z)
Vec3 = Tuple[float, float, float]
Mat3 = Tuple[Vec3, Vec3, Vec3]

# Rotation of pi about x: maps FLU <-> FRD (and an FLU-like world frame <-> FRD world frame).
Q_FLU_TO_FRD: Quat = (0.0, 1.0, 0.0, 0.0)
# ENU -> NED: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu. As a quaternion this is
# (0, sqrt(1/2), sqrt(1/2), 0).
_S = math.sqrt(0.5)
Q_ENU_TO_NED: Quat = (0.0, _S, _S, 0.0)


# --------------------------------------------------------------------------- quaternion algebra
def q_mul(a: Quat, b: Quat) -> Quat:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q: Quat) -> Quat:
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q: Quat) -> float:
    return math.sqrt(sum(c * c for c in q))


def q_normalize(q: Quat) -> Quat:
    n = q_norm(q)
    if not math.isfinite(n) or n < 1e-9:
        raise ValueError(f"cannot normalize quaternion {q!r}")
    w, x, y, z = (c / n for c in q)
    # Canonical sign (w >= 0) keeps outputs stable for tests/logs; q and -q are the same rotation.
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (w, x, y, z)


def q_rotate(q: Quat, v: Sequence[float]) -> Vec3:
    p = (0.0, float(v[0]), float(v[1]), float(v[2]))
    r = q_mul(q_mul(q, p), q_conj(q))
    return (r[1], r[2], r[3])


def q_from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """Intrinsic Z-Y-X (yaw, then pitch, then roll), same as ROS tf2 ``setRPY``."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def q_to_matrix(q: Quat) -> Mat3:
    w, x, y, z = q_normalize(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    )


def ros_xyzw_to_wxyz(x: float, y: float, z: float, w: float) -> Quat:
    return (w, x, y, z)


# --------------------------------------------------------------------------- rigid transforms
def compose(p_ab: Vec3, q_ab: Quat, p_bc: Vec3, q_bc: Quat) -> Tuple[Vec3, Quat]:
    """T_ac = T_ab * T_bc."""
    r = q_rotate(q_ab, p_bc)
    return (p_ab[0] + r[0], p_ab[1] + r[1], p_ab[2] + r[2]), q_mul(q_ab, q_bc)


def invert(p_ab: Vec3, q_ab: Quat) -> Tuple[Vec3, Quat]:
    q_ba = q_conj(q_ab)
    r = q_rotate(q_ba, p_ab)
    return (-r[0], -r[1], -r[2]), q_ba


def reexpress_in_vehicle_frame(
    p_w_imu: Vec3, q_w_imu: Quat, p_imu_base: Vec3, q_imu_base: Quat
) -> Tuple[Vec3, Quat]:
    """Pose of base_link relative to "base_link at FAST-LIO start-up".

    FAST-LIO's world frame (``camera_init``) is the IMU frame at initialisation. If the Mid-360
    is mounted rotated (upside-down, pitched, yawed) relative to the airframe, that world frame
    is rotated too. Re-expressing it as ``T_base0_base(t) = T_base_imu * T_w_imu(t) * T_imu_base``
    gives a world frame aligned with the vehicle's FLU axes at start-up (heading arbitrary,
    level only if the vehicle sat level when FAST-LIO initialised).
    """
    p_base_imu, q_base_imu = invert(p_imu_base, q_imu_base)
    p1, q1 = compose(p_base_imu, q_base_imu, p_w_imu, q_w_imu)
    return compose(p1, q1, p_imu_base, q_imu_base)


# --------------------------------------------------------------------------- ROS -> PX4
def flu_world_to_frd(p: Vec3, q_world_body_flu: Quat) -> Tuple[Vec3, Quat]:
    """FLU-like world + FLU body  ->  FRD world + FRD body (PX4 ``POSE_FRAME_FRD``).

    p' = F p with F = diag(1, -1, -1); R' = F R F.
    """
    p_out = (p[0], -p[1], -p[2])
    q_out = q_mul(q_mul(Q_FLU_TO_FRD, q_world_body_flu), q_conj(Q_FLU_TO_FRD))
    return p_out, q_normalize(q_out)


def enu_world_to_ned(p: Vec3, q_world_body_flu: Quat) -> Tuple[Vec3, Quat]:
    """ENU world + FLU body  ->  NED world + FRD body (PX4 ``POSE_FRAME_NED``)."""
    p_out = (p[1], p[0], -p[2])
    q_out = q_mul(q_mul(Q_ENU_TO_NED, q_world_body_flu), q_conj(Q_FLU_TO_FRD))
    return p_out, q_normalize(q_out)


def flu_vector_to_frd(v: Vec3) -> Vec3:
    return (v[0], -v[1], -v[2])


def enu_vector_to_ned(v: Vec3) -> Vec3:
    return (v[1], v[0], -v[2])


def variance_flu_to_frd(var: Vec3) -> Vec3:
    # Axis sign flips do not change a diagonal variance.
    return (var[0], var[1], var[2])


def variance_enu_to_ned(var: Vec3) -> Vec3:
    return (var[1], var[0], var[2])


# --------------------------------------------------------------------------- covariance helpers
def diag_from_ros_covariance(cov: Sequence[float]) -> Tuple[Vec3, Vec3]:
    """REP-conformant 6x6 row-major covariance -> (position var, orientation var)."""
    if len(cov) != 36:
        raise ValueError("expected a 36-element covariance")
    return (cov[0], cov[7], cov[14]), (cov[21], cov[28], cov[35])


def diag_from_fastlio_covariance(cov: Sequence[float]) -> Tuple[Vec3, Vec3]:
    """FAST_LIO_ROS2 fills ``pose.covariance`` with its state order (pos, rot) swapped.

    In ``laserMapping.cpp::publish_odometry`` (ros2 branch, commit 2fffc57) the loop maps row
    ``i`` to state row ``k = i + 3`` for ``i < 3`` and ``k = i - 3`` otherwise, then writes
    ``P(k, 3..5)`` into columns 0..2 and ``P(k, 0..2)`` into columns 3..5. With the IKFoM state
    order ``(pos, rot, ...)`` this puts the rotation variances at indices 0/7/14 and the
    position variances at 21/28/35. It is also written *after*
    ``publish()`` so each message carries the previous cycle's covariance. Use with care.
    """
    if len(cov) != 36:
        raise ValueError("expected a 36-element covariance")
    return (cov[21], cov[28], cov[35]), (cov[0], cov[7], cov[14])


def is_finite_vec(v: Sequence[float]) -> bool:
    return all(math.isfinite(c) for c in v)
