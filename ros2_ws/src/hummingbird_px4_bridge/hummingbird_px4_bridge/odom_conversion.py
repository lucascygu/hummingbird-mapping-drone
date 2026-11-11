# Generated / assisted by Grok
"""ROS-free core of the bridge: turn one FAST-LIO odometry sample into VehicleOdometry fields.

Kept separate from the ROS node so that pytest can exercise it without rclpy / px4_msgs.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

from hummingbird_px4_bridge import frames

NAN = float("nan")

POSE_FRAME_NED = 1
POSE_FRAME_FRD = 2
VELOCITY_FRAME_UNKNOWN = 0
VELOCITY_FRAME_NED = 1
VELOCITY_FRAME_FRD = 2


@dataclass
class BridgeConfig:
    # "frd": FAST-LIO world is an FLU-like frame with arbitrary heading (default, correct for
    #        stock FAST-LIO). "ned": input world is a true ENU frame (e.g. after GNSS alignment).
    output_frame: str = "frd"
    # Static transform IMU(body) -> vehicle base_link (FLU). Position of base_link origin in the
    # IMU frame and its orientation. PLACEHOLDER: identity = Mid-360 level, cable to the rear,
    # IMU at the vehicle centre. Measure on the real airframe.
    imu_to_base_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_to_base_rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    reexpress_world_in_vehicle_frame: bool = True
    # "fixed" | "fastlio" | "ros"
    covariance_source: str = "fixed"
    fixed_position_variance: float = 0.01  # m^2  (0.1 m 1-sigma)
    fixed_orientation_variance: float = 0.0025  # rad^2 (~2.9 deg 1-sigma)
    min_position_variance: float = 1e-4
    min_orientation_variance: float = 1e-5
    # Velocity: FAST_LIO_ROS2 never fills twist; optional finite-difference estimate.
    estimate_velocity: bool = False
    velocity_lpf_alpha: float = 0.5
    # A jump larger than this between consecutive samples bumps reset_counter.
    reset_jump_m: float = 1.0
    quality: int = 0  # 0 = unknown (EKF2_EV_QMIN=0 ignores it)


@dataclass
class OdometryFields:
    timestamp_sample_us: int
    pose_frame: int
    position: Tuple[float, float, float]
    q: Tuple[float, float, float, float]
    velocity_frame: int
    velocity: Tuple[float, float, float]
    angular_velocity: Tuple[float, float, float]
    position_variance: Tuple[float, float, float]
    orientation_variance: Tuple[float, float, float]
    velocity_variance: Tuple[float, float, float]
    reset_counter: int
    quality: int


@dataclass
class _State:
    last_pos: Optional[Tuple[float, float, float]] = None
    last_t_us: Optional[int] = None
    vel: Optional[Tuple[float, float, float]] = None
    reset_counter: int = 0


def choose_timestamp_us(
    header_stamp_ns: int, now_ns: int, mode: str = "auto", max_skew_s: float = 0.5,
    receipt_latency_ms: float = 0.0,
) -> Tuple[int, bool]:
    """Return (timestamp_sample_us, used_header).

    PX4's uXRCE-DDS client subtracts the agent<->FMU clock offset from ``timestamp`` and
    ``timestamp_sample`` (Tools/msg/templates/ucdr/msg.h.em), so they must be in the companion's
    ROS clock. Livox points are stamped by the LiDAR; without PTP/gPTP sync that clock is NOT the
    Jetson clock, so ``auto`` falls back to receipt time when the header is implausible.
    """
    receipt_us = int(now_ns // 1000 - receipt_latency_ms * 1000.0)
    if mode == "receipt":
        return receipt_us, False
    header_us = int(header_stamp_ns // 1000)
    if mode == "header":
        return header_us, True
    if header_stamp_ns > 0 and abs(now_ns - header_stamp_ns) <= max_skew_s * 1e9:
        return header_us, True
    return receipt_us, False


class OdometryConverter:
    def __init__(self, config: BridgeConfig):
        if config.output_frame not in ("frd", "ned"):
            raise ValueError("output_frame must be 'frd' or 'ned'")
        if config.covariance_source not in ("fixed", "fastlio", "ros"):
            raise ValueError("covariance_source must be fixed|fastlio|ros")
        self.cfg = config
        self._q_imu_base = frames.q_normalize(frames.q_from_rpy(*config.imu_to_base_rpy))
        self._p_imu_base = tuple(float(c) for c in config.imu_to_base_xyz)
        self._s = _State()

    @property
    def reset_counter(self) -> int:
        return self._s.reset_counter

    def bump_reset(self) -> None:
        self._s.reset_counter = (self._s.reset_counter + 1) % 256
        self._s.last_pos = None
        self._s.last_t_us = None
        self._s.vel = None

    def _variances(self, cov: Optional[Sequence[float]]):
        c = self.cfg
        if c.covariance_source == "fixed" or cov is None or len(cov) != 36:
            pv = (c.fixed_position_variance,) * 3
            ov = (c.fixed_orientation_variance,) * 3
        elif c.covariance_source == "fastlio":
            pv, ov = frames.diag_from_fastlio_covariance(cov)
        else:
            pv, ov = frames.diag_from_ros_covariance(cov)
        if not (frames.is_finite_vec(pv) and frames.is_finite_vec(ov)) or min(pv) <= 0 or min(ov) <= 0:
            # FAST-LIO's very first message has an all-zero covariance; fall back.
            pv = (c.fixed_position_variance,) * 3
            ov = (c.fixed_orientation_variance,) * 3
        pv = tuple(max(v, c.min_position_variance) for v in pv)
        ov = tuple(max(v, c.min_orientation_variance) for v in ov)
        return pv, ov

    def convert(
        self,
        t_us: int,
        position: Sequence[float],
        q_xyzw: Sequence[float],
        covariance: Optional[Sequence[float]] = None,
        angular_velocity_flu: Optional[Sequence[float]] = None,
    ) -> Optional[OdometryFields]:
        """Convert one sample. Returns None if the input is invalid (NaN / zero quaternion)."""
        p = tuple(float(c) for c in position)
        q_in = frames.ros_xyzw_to_wxyz(*[float(c) for c in q_xyzw])
        if not frames.is_finite_vec(p) or not frames.is_finite_vec(q_in):
            return None
        try:
            q_in = frames.q_normalize(q_in)
        except ValueError:
            return None

        if self.cfg.reexpress_world_in_vehicle_frame:
            p_w, q_w = frames.reexpress_in_vehicle_frame(p, q_in, self._p_imu_base, self._q_imu_base)
        else:
            p_w, q_w = frames.compose(p, q_in, self._p_imu_base, self._q_imu_base)

        pv, ov = self._variances(covariance)
        if self.cfg.output_frame == "frd":
            pos_out, q_out = frames.flu_world_to_frd(p_w, q_w)
            pv_out = frames.variance_flu_to_frd(pv)
            pose_frame, vel_frame = POSE_FRAME_FRD, VELOCITY_FRAME_FRD
        else:
            pos_out, q_out = frames.enu_world_to_ned(p_w, q_w)
            pv_out = frames.variance_enu_to_ned(pv)
            pose_frame, vel_frame = POSE_FRAME_NED, VELOCITY_FRAME_NED
        # Orientation variance is about body axes (roll, pitch, yaw); FLU->FRD keeps magnitudes.
        ov_out = ov

        s = self._s
        if s.last_pos is not None:
            jump = math.dist(s.last_pos, pos_out)
            if jump > self.cfg.reset_jump_m or (s.last_t_us is not None and t_us <= s.last_t_us):
                self.bump_reset()

        velocity = (NAN, NAN, NAN)
        vel_var = (NAN, NAN, NAN)
        velocity_frame = VELOCITY_FRAME_UNKNOWN
        if self.cfg.estimate_velocity and s.last_pos is not None and s.last_t_us is not None:
            dt = (t_us - s.last_t_us) * 1e-6
            if 1e-3 < dt < 0.5:
                raw = tuple((a - b) / dt for a, b in zip(pos_out, s.last_pos))
                a = self.cfg.velocity_lpf_alpha
                s.vel = raw if s.vel is None else tuple(a * r + (1 - a) * v for r, v in zip(raw, s.vel))
                velocity = s.vel
                # Crude: differencing two positions with variance pv over dt.
                vel_var = tuple(2.0 * v / (dt * dt) for v in pv_out)
                velocity_frame = vel_frame
        s.last_pos = pos_out
        s.last_t_us = t_us

        if angular_velocity_flu is not None and frames.is_finite_vec(angular_velocity_flu):
            # Body rates of the IMU frame -> base_link FLU -> FRD.
            w_base = frames.q_rotate(frames.q_conj(self._q_imu_base), angular_velocity_flu)
            ang = frames.flu_vector_to_frd(w_base)
        else:
            ang = (NAN, NAN, NAN)

        return OdometryFields(
            timestamp_sample_us=int(t_us),
            pose_frame=pose_frame,
            position=pos_out,
            q=q_out,
            velocity_frame=velocity_frame,
            velocity=velocity,
            angular_velocity=ang,
            position_variance=pv_out,
            orientation_variance=ov_out,
            velocity_variance=vel_var,
            reset_counter=s.reset_counter,
            quality=int(self.cfg.quality),
        )
