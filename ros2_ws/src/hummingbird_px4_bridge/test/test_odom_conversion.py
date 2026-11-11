# Generated / assisted by Grok
# Extended by Claude: discontinuity handling, sticky time source, variance rotation
import math

import pytest

from hummingbird_px4_bridge.odom_conversion import (
    POSE_FRAME_FRD, POSE_FRAME_NED, VELOCITY_FRAME_FRD, VELOCITY_FRAME_UNKNOWN,
    BridgeConfig, OdometryConverter, StampSelector, choose_timestamp_us,
)

ID_XYZW = (0.0, 0.0, 0.0, 1.0)


def test_default_frd_output():
    c = OdometryConverter(BridgeConfig())
    o = c.convert(1_000_000, (1.0, 2.0, 3.0), ID_XYZW)
    assert o.pose_frame == POSE_FRAME_FRD
    assert o.position == pytest.approx((1.0, -2.0, -3.0))
    assert o.q == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert all(math.isnan(v) for v in o.velocity)
    assert o.velocity_frame == VELOCITY_FRAME_UNKNOWN
    assert o.position_variance == (0.01, 0.01, 0.01)
    assert o.timestamp_sample_us == 1_000_000
    assert o.reset_counter == 0


def test_ned_mode():
    c = OdometryConverter(BridgeConfig(output_frame="ned"))
    o = c.convert(1, (1.0, 2.0, 3.0), ID_XYZW)
    assert o.pose_frame == POSE_FRAME_NED
    assert o.position == pytest.approx((2.0, 1.0, -3.0))


def test_invalid_inputs_dropped():
    c = OdometryConverter(BridgeConfig())
    assert c.convert(1, (float("nan"), 0, 0), ID_XYZW) is None
    assert c.convert(1, (0, 0, 0), (0, 0, 0, 0)) is None


def test_zero_fastlio_covariance_falls_back_to_fixed():
    c = OdometryConverter(BridgeConfig(covariance_source="fastlio"))
    o = c.convert(1, (0, 0, 0), ID_XYZW, covariance=[0.0] * 36)
    assert o.position_variance == (0.01, 0.01, 0.01)


def test_fastlio_covariance_used_and_floored():
    cov = [0.0] * 36
    cov[0] = cov[7] = cov[14] = 1e-9    # rot, below floor
    cov[21], cov[28], cov[35] = 0.04, 0.09, 0.16
    c = OdometryConverter(BridgeConfig(covariance_source="fastlio"))
    o = c.convert(1, (0, 0, 0), ID_XYZW, covariance=cov)
    assert o.position_variance == pytest.approx((0.04, 0.09, 0.16))
    assert o.orientation_variance == pytest.approx((1e-5,) * 3)


MS = 1_000          # microseconds
S = 1_000_000


def feed(c, t_us, x, y=0.0, z=0.0):
    return c.convert(t_us, (x, y, z), ID_XYZW)


def test_small_steps_and_fast_flight_do_not_reset():
    c = OdometryConverter(BridgeConfig())
    assert feed(c, 0, 0.0).reset_counter == 0
    assert feed(c, 100 * MS, 0.5).reset_counter == 0          # 5 m/s
    assert feed(c, 200 * MS, 2.5).reset_counter == 0          # 20 m/s, 2 m step: fast but plausible
    # 0.6 s without messages at 5 m/s: a 3 m step, still the same session
    assert feed(c, 800 * MS, 5.5).reset_counter == 0
    assert c.stats["published"] == 4 and c.stats["glitches"] == 0


def test_implausible_jump_goes_silent_then_resumes_as_new_session():
    c = OdometryConverter(BridgeConfig(holdoff_s=1.0))
    assert feed(c, 0, 0.0) is not None
    assert feed(c, 100 * MS, 0.1) is not None
    # 50 m in 0.1 s: FAST-LIO diverged or re-initialised. PX4 must NOT be told to follow it.
    assert feed(c, 200 * MS, 50.0) is None
    assert c.last_drop_reason == "holdoff" and c.holding and c.reset_counter == 0
    # steady again, but still inside the hold-off window
    for k in range(3, 12):
        assert feed(c, k * 100 * MS, 50.0 + 0.01 * k) is None
    # 1.0 s after the jump: resume, flagged as a new session
    o = feed(c, 1200 * MS, 50.2)
    assert o is not None and o.reset_counter == 1
    assert c.last_session_event == "glitch_recovered" and not c.holding
    assert feed(c, 1300 * MS, 50.3).reset_counter == 1
    assert c.stats["glitches"] == 1 and c.stats["held"] == 10


def test_stream_that_keeps_jumping_is_never_forwarded():
    c = OdometryConverter(BridgeConfig(holdoff_s=1.0))
    feed(c, 0, 0.0)
    for k in range(1, 60):                       # 6 s of 40 m steps every 0.1 s
        assert feed(c, k * 100 * MS, 40.0 * k) is None
    assert c.reset_counter == 0 and c.stats["published"] == 1


def test_gap_starts_a_new_session():
    c = OdometryConverter(BridgeConfig(gap_reset_s=1.0))
    assert feed(c, 0, 0.0).reset_counter == 0
    o = feed(c, 3 * S, 0.2)                      # the source stalled for 3 s
    assert o is not None and o.reset_counter == 1 and c.last_session_event == "gap"
    assert feed(c, 3 * S + 100 * MS, 0.3).reset_counter == 1
    assert c.stats["gaps"] == 1


def test_out_of_order_samples_are_dropped_without_reset():
    c = OdometryConverter(BridgeConfig())
    assert feed(c, 400 * MS, 0.0) is not None
    assert feed(c, 350 * MS, 0.0) is None and c.last_drop_reason == "out_of_order"   # went back 50 ms
    assert feed(c, 400 * MS, 0.0) is None                                            # duplicate stamp
    assert feed(c, 500 * MS, 0.1).reset_counter == 0
    assert c.stats["out_of_order"] == 2


def test_clock_step_backwards_starts_a_new_session():
    c = OdometryConverter(BridgeConfig(clock_step_s=1.0))
    assert feed(c, 100 * S, 0.0).reset_counter == 0
    o = feed(c, 40 * S, 0.1)                     # companion clock stepped back by a minute
    assert o is not None and o.reset_counter == 1 and c.last_session_event == "clock_step"
    assert feed(c, 40 * S + 100 * MS, 0.2).reset_counter == 1


def test_zero_holdoff_resets_immediately():
    c = OdometryConverter(BridgeConfig(holdoff_s=0.0))
    feed(c, 0, 0.0)
    o = feed(c, 100 * MS, 50.0)
    assert o is not None and o.reset_counter == 1


def test_reset_counter_wraps_at_256():
    c = OdometryConverter(BridgeConfig())
    for _ in range(256):
        c.bump_reset()
    assert c.reset_counter == 0


def test_absurd_position_is_invalid():
    c = OdometryConverter(BridgeConfig(max_abs_position_m=1000.0))
    assert feed(c, 0, 1500.0) is None and c.last_drop_reason == "invalid"
    assert feed(c, 100 * MS, 1.0) is not None
    assert c.stats["invalid"] == 1


def test_velocity_is_not_differenced_across_sessions():
    c = OdometryConverter(BridgeConfig(estimate_velocity=True, velocity_lpf_alpha=1.0))
    feed(c, 0, 0.0)
    assert feed(c, 100 * MS, 0.1).velocity == pytest.approx((1.0, 0.0, 0.0))
    o = feed(c, 5 * S, 3.0)                      # new session after a gap
    assert o.reset_counter == 1 and all(math.isnan(v) for v in o.velocity)
    assert o.velocity_frame == VELOCITY_FRAME_UNKNOWN


def test_velocity_estimate_in_frd():
    c = OdometryConverter(BridgeConfig(estimate_velocity=True, velocity_lpf_alpha=1.0))
    c.convert(0, (0, 0, 0), ID_XYZW)
    o = c.convert(100_000, (0.1, 0.2, 0.3), ID_XYZW)  # 0.1 s later
    assert o.velocity_frame == VELOCITY_FRAME_FRD
    assert o.velocity == pytest.approx((1.0, -2.0, -3.0))
    assert all(v > 0 for v in o.velocity_variance)


def test_upside_down_mount_in_bridge():
    c = OdometryConverter(BridgeConfig(imu_to_base_rpy=(math.pi, 0.0, 0.0)))
    o = c.convert(1, (0.0, 0.0, -2.0), ID_XYZW)  # inverted IMU sees climb as -z
    assert o.position == pytest.approx((0.0, 0.0, -2.0), abs=1e-9)  # FRD: up is -z
    assert o.q == pytest.approx((1.0, 0.0, 0.0, 0.0), abs=1e-9)


def test_bad_config_rejected():
    with pytest.raises(ValueError):
        OdometryConverter(BridgeConfig(output_frame="enu"))
    with pytest.raises(ValueError):
        OdometryConverter(BridgeConfig(covariance_source="magic"))
    for bad in (dict(holdoff_s=-1.0), dict(gap_reset_s=0.0), dict(reset_jump_m=0.0),
                dict(max_plausible_speed_mps=0.0), dict(clock_step_s=0.0)):
        with pytest.raises(ValueError):
            OdometryConverter(BridgeConfig(**bad))


def ros_cov(pos_var, rot_var):
    cov = [0.0] * 36
    cov[0], cov[7], cov[14] = pos_var
    cov[21], cov[28], cov[35] = rot_var
    return cov


def test_variances_follow_a_rotated_mount():
    cov = ros_cov((0.04, 0.09, 0.16), (0.001, 0.002, 0.003))
    # Mid-360 yawed 90 deg on the airframe: the IMU's x axis is the vehicle's y axis.
    c = OdometryConverter(BridgeConfig(covariance_source="ros", imu_to_base_rpy=(0.0, 0.0, math.pi / 2)))
    o = c.convert(1, (0, 0, 0), ID_XYZW, covariance=cov)
    assert o.position_variance == pytest.approx((0.09, 0.04, 0.16))
    assert o.orientation_variance == pytest.approx((0.002, 0.001, 0.003))
    # Upside-down mount only flips signs: variances stay on their axes.
    c = OdometryConverter(BridgeConfig(covariance_source="ros", imu_to_base_rpy=(math.pi, 0.0, 0.0)))
    o = c.convert(1, (0, 0, 0), ID_XYZW, covariance=cov)
    assert o.position_variance == pytest.approx((0.04, 0.09, 0.16))
    # Without re-expression the world frame is untouched; only the body-axis attitude error turns.
    c = OdometryConverter(BridgeConfig(covariance_source="ros", imu_to_base_rpy=(0.0, 0.0, math.pi / 2),
                                       reexpress_world_in_vehicle_frame=False))
    o = c.convert(1, (0, 0, 0), ID_XYZW, covariance=cov)
    assert o.position_variance == pytest.approx((0.04, 0.09, 0.16))
    assert o.orientation_variance == pytest.approx((0.002, 0.001, 0.003))


def test_timestamp_selection():
    now = 1_700_000_000_000_000_000
    assert choose_timestamp_us(now - 50_000_000, now) == ((now - 50_000_000) // 1000, True)
    # LiDAR clock since power-on (no PTP): falls back to receipt time
    assert choose_timestamp_us(12_000_000_000, now) == (now // 1000, False)
    assert choose_timestamp_us(12_000_000_000, now, mode="header") == (12_000_000, True)
    assert choose_timestamp_us(now, now, mode="receipt", receipt_latency_ms=20) == (now // 1000 - 20_000, False)


NOW = 1_700_000_000_000_000_000


def test_stamp_selector_sticks_to_its_source():
    sel = StampSelector("auto", max_skew_s=0.5, switch_after=3)
    good = NOW - 50_000_000                      # header 50 ms old: plausible
    bad = NOW - 600_000_000                      # header 0.6 s old: beyond the skew limit
    assert sel.select(good, NOW) == (good // 1000, True, True)       # first sample decides
    assert sel.select(bad, NOW) == (bad // 1000, True, False)        # one outlier: keep header
    assert sel.select(good, NOW) == (good // 1000, True, False)
    assert sel.select(bad, NOW)[1:] == (True, False)
    assert sel.select(bad, NOW)[1:] == (True, False)
    assert sel.select(bad, NOW) == (NOW // 1000, False, True)        # 3 in a row: switch, once
    assert sel.select(bad, NOW) == (NOW // 1000, False, False)
    assert sel.select(good, NOW)[1:] == (False, False)               # and it stays on receipt
    assert sel.use_header is False


def test_stamp_selector_unsynced_lidar_uses_receipt_from_the_start():
    sel = StampSelector("auto", receipt_latency_ms=20.0)
    t_us, used_header, changed = sel.select(12_000_000_000, NOW)     # LiDAR time since power-on
    assert (t_us, used_header, changed) == (NOW // 1000 - 20_000, False, True)


def test_stamp_selector_fixed_modes():
    assert StampSelector("header").select(12_000_000_000, NOW)[:2] == (12_000_000, True)
    assert StampSelector("receipt").select(NOW - 1, NOW)[:2] == (NOW // 1000, False)
    with pytest.raises(ValueError):
        StampSelector("gps")
