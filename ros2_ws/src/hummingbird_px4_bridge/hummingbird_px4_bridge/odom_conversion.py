# Generated / assisted by Grok
# Reviewed and extended by Claude (discontinuity handling, sticky time source, variance rotation)
"""ROS-free core of the bridge: turn one FAST-LIO odometry sample into VehicleOdometry fields.

Kept separate from the ROS node so that pytest can exercise it without rclpy / px4_msgs.

Discontinuities
---------------
In PX4 (EKF2, checked against v1.16.1 ``EKF/aid_sources/external_vision``) a change of
``reset_counter`` makes the estimator jump to the external-vision pose at once, without testing
it against its innovation gate. So ``reset_counter`` must never be bumped for a sample that is
merely implausible: a single outlier would yank the estimate away and back, and a diverging
FAST-LIO would drag the flight controller along frame by frame. This converter therefore:

* drops out-of-order / duplicate samples (PX4 ignores them anyway);
* treats an implausible jump as a fault: it stops publishing for ``holdoff_s`` (long enough for
  EKF2 to time external vision out) and only then resumes as a new session (``reset_counter``+1).
  While the stream keeps jumping, nothing is published at all;
* starts a new session after a gap longer than ``gap_reset_s`` (source stalled or restarted) or
  after the clock stepped backwards.

This is damage limitation, not a cure. When external vision is PX4's only position source,
EKF2 itself re-aligns to it after about one second of rejected measurements, so a FAST-LIO
that has settled on a wrong pose will still be believed. Only an independent source (GNSS,
the pilot) can catch that.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

from hummingbird_px4_bridge import frames

NAN = float("nan")

# px4_msgs/VehicleOdometry constants (identical in PX4 v1.15 and v1.16).
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
    # A step is a discontinuity only if it is longer than reset_jump_m AND faster than
    # max_plausible_speed_mps (so a message gap during normal flight is not mistaken for one).
    reset_jump_m: float = 1.0
    max_plausible_speed_mps: float = 30.0
    # After a discontinuity nothing is published for this long. Must exceed EKF2's external
    # vision timeout (2 x EV_MAX_INTERVAL = 0.4 s) so PX4 has dropped the old session first.
    holdoff_s: float = 1.0
    # No sample for this long = the source stalled or restarted: resume as a new session.
    gap_reset_s: float = 1.0
    # Timestamps going back by more than this are a clock step, not message reordering.
    clock_step_s: float = 1.0
    # Positions beyond this are treated as garbage (also keeps float32 resolution sane).
    max_abs_position_m: float = 10_000.0
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
    hold_until_us: Optional[int] = None


def _new_stats() -> Dict[str, int]:
    return {"published": 0, "invalid": 0, "out_of_order": 0, "held": 0,
            "glitches": 0, "gaps": 0, "clock_steps": 0}


def choose_timestamp_us(
    header_stamp_ns: int, now_ns: int, mode: str = "auto", max_skew_s: float = 0.5,
    receipt_latency_ms: float = 0.0,
) -> Tuple[int, bool]:
    """Return (timestamp_sample_us, used_header). Stateless; see ``StampSelector`` for the node.

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


class StampSelector:
    """``choose_timestamp_us`` with memory.

    In ``auto`` mode the stateless choice can flip between header and receipt time from one
    sample to the next when the header skew sits near ``max_skew_s``. Every flip makes the time
    series jump by up to that skew, which EKF2 sees as out-of-order or missing data. This class
    keeps the current source until the other one has been the better choice for
    ``switch_after`` consecutive samples.
    """

    def __init__(self, mode: str = "auto", max_skew_s: float = 0.5,
                 receipt_latency_ms: float = 0.0, switch_after: int = 20):
        if mode not in ("auto", "header", "receipt"):
            raise ValueError("stamp_mode must be auto|header|receipt")
        self.mode = mode
        self.max_skew_s = float(max_skew_s)
        self.receipt_latency_ms = float(receipt_latency_ms)
        self.switch_after = max(1, int(switch_after))
        self.use_header: Optional[bool] = None  # None until the first sample
        self._disagree = 0

    def select(self, header_stamp_ns: int, now_ns: int) -> Tuple[int, bool, bool]:
        """Return (timestamp_sample_us, used_header, source_changed)."""
        if self.mode != "auto":
            t_us, used = choose_timestamp_us(header_stamp_ns, now_ns, self.mode,
                                             self.max_skew_s, self.receipt_latency_ms)
            changed = self.use_header is None
            self.use_header = used
            return t_us, used, changed
        _, want_header = choose_timestamp_us(header_stamp_ns, now_ns, "auto", self.max_skew_s)
        changed = False
        if self.use_header is None:
            self.use_header, changed = want_header, True
        elif want_header != self.use_header:
            self._disagree += 1
            if self._disagree >= self.switch_after:
                self.use_header, self._disagree, changed = want_header, 0, True
        else:
            self._disagree = 0
        t_us, _ = choose_timestamp_us(header_stamp_ns, now_ns,
                                      "header" if self.use_header else "receipt",
                                      self.max_skew_s, self.receipt_latency_ms)
        return t_us, bool(self.use_header), changed


class OdometryConverter:
    def __init__(self, config: BridgeConfig):
        if config.output_frame not in ("frd", "ned"):
            raise ValueError("output_frame must be 'frd' or 'ned'")
        if config.covariance_source not in ("fixed", "fastlio", "ros"):
            raise ValueError("covariance_source must be fixed|fastlio|ros")
        if config.holdoff_s < 0 or config.gap_reset_s <= 0 or config.reset_jump_m <= 0 \
                or config.max_plausible_speed_mps <= 0 or config.clock_step_s <= 0:
            raise ValueError("holdoff_s must be >= 0; gap_reset_s, reset_jump_m, "
                             "max_plausible_speed_mps and clock_step_s must be > 0")
        self.cfg = config
        self._q_imu_base = frames.q_normalize(frames.q_from_rpy(*config.imu_to_base_rpy))
        self._p_imu_base = tuple(float(c) for c in config.imu_to_base_xyz)
        self._s = _State()
        self.stats: Dict[str, int] = _new_stats()
        # Why the last sample was not published ("" if it was): invalid | out_of_order | holdoff.
        self.last_drop_reason = ""
        # Set when the last call opened a new session: gap | clock_step | glitch_recovered.
        self.last_session_event = ""

    @property
    def reset_counter(self) -> int:
        return self._s.reset_counter

    @property
    def holding(self) -> bool:
        return self._s.hold_until_us is not None

    def bump_reset(self) -> None:
        self._s.reset_counter = (self._s.reset_counter + 1) % 256
        self._s.last_pos = None
        self._s.last_t_us = None
        self._s.vel = None
        self._s.hold_until_us = None

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
        q_base_imu = frames.q_conj(self._q_imu_base)
        if c.reexpress_world_in_vehicle_frame:
            # The world frame was turned from "IMU at start-up" to "base_link at start-up".
            pv = frames.rotate_variance(pv, q_base_imu)
        # Attitude error is about the IMU body axes; express it about base_link axes.
        ov = frames.rotate_variance(ov, q_base_imu)
        pv = tuple(max(v, c.min_position_variance) for v in pv)
        ov = tuple(max(v, c.min_orientation_variance) for v in ov)
        return pv, ov

    def _drop(self, reason: str, counter: str) -> None:
        self.stats[counter] += 1
        self.last_drop_reason = reason

    def _continuity(self, t_us: int, pos_out: Tuple[float, float, float]) -> bool:
        """Session bookkeeping for one valid sample. Returns True if it may be published."""
        s, c = self._s, self.cfg
        if s.last_t_us is None or s.last_pos is None:
            return True  # first sample of a session

        dt = (t_us - s.last_t_us) * 1e-6
        if dt <= 0.0:
            if -dt <= c.clock_step_s:
                self._drop("out_of_order", "out_of_order")
                return False
            # The companion clock stepped backwards (e.g. NTP). Nothing to compare against.
            self.stats["clock_steps"] += 1
            self.bump_reset()
            self.last_session_event = "clock_step"
            return True

        jump = math.dist(s.last_pos, pos_out)
        implausible = jump > c.reset_jump_m and jump / dt > c.max_plausible_speed_mps
        if implausible:
            # Fault: go silent, keep following the stream, resume only once it is steady again.
            self.stats["glitches"] += 1
            s.hold_until_us = t_us + int(c.holdoff_s * 1e6)
            s.vel = None
        elif s.hold_until_us is None and dt > c.gap_reset_s:
            # The source stalled or was restarted; EKF2 has timed external vision out already.
            self.stats["gaps"] += 1
            self.bump_reset()
            self.last_session_event = "gap"
            return True

        if s.hold_until_us is not None:
            if t_us < s.hold_until_us:
                s.last_pos, s.last_t_us = pos_out, t_us
                self._drop("holdoff", "held")
                return False
            self.bump_reset()
            self.last_session_event = "glitch_recovered"
        return True

    def convert(
        self,
        t_us: int,
        position: Sequence[float],
        q_xyzw: Sequence[float],
        covariance: Optional[Sequence[float]] = None,
        angular_velocity_flu: Optional[Sequence[float]] = None,
    ) -> Optional[OdometryFields]:
        """Convert one sample.

        Returns None when nothing must be sent to PX4 for it; ``last_drop_reason`` says why
        (``invalid`` input, ``out_of_order`` timestamp, or ``holdoff`` after a discontinuity).
        """
        self.last_drop_reason = ""
        self.last_session_event = ""
        p = tuple(float(c) for c in position)
        q_in = frames.ros_xyzw_to_wxyz(*[float(c) for c in q_xyzw])
        if not frames.is_finite_vec(p) or not frames.is_finite_vec(q_in) \
                or max(abs(c) for c in p) > self.cfg.max_abs_position_m:
            self._drop("invalid", "invalid")
            return None
        try:
            q_in = frames.q_normalize(q_in)
        except ValueError:
            self._drop("invalid", "invalid")
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

        if not self._continuity(int(t_us), pos_out):
            return None

        s = self._s
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
        s.last_t_us = int(t_us)

        if angular_velocity_flu is not None and frames.is_finite_vec(angular_velocity_flu):
            # Body rates of the IMU frame -> base_link FLU -> FRD.
            w_base = frames.q_rotate(frames.q_conj(self._q_imu_base), angular_velocity_flu)
            ang = frames.flu_vector_to_frd(w_base)
        else:
            ang = (NAN, NAN, NAN)

        self.stats["published"] += 1
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
