# Generated / assisted by Grok
import math

import pytest

from hummingbird_px4_bridge.odom_conversion import (
    POSE_FRAME_FRD, POSE_FRAME_NED, VELOCITY_FRAME_FRD, VELOCITY_FRAME_UNKNOWN,
    BridgeConfig, OdometryConverter, choose_timestamp_us,
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


def test_reset_counter_on_jump_and_time_reversal():
    c = OdometryConverter(BridgeConfig(reset_jump_m=1.0))
    assert c.convert(100, (0, 0, 0), ID_XYZW).reset_counter == 0
    assert c.convert(200, (0.5, 0, 0), ID_XYZW).reset_counter == 0
    assert c.convert(300, (5.0, 0, 0), ID_XYZW).reset_counter == 1
    assert c.convert(400, (5.1, 0, 0), ID_XYZW).reset_counter == 1
    assert c.convert(350, (5.1, 0, 0), ID_XYZW).reset_counter == 2  # time went backwards


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


def test_timestamp_selection():
    now = 1_700_000_000_000_000_000
    assert choose_timestamp_us(now - 50_000_000, now) == ((now - 50_000_000) // 1000, True)
    # LiDAR clock since power-on (no PTP): falls back to receipt time
    assert choose_timestamp_us(12_000_000_000, now) == (now // 1000, False)
    assert choose_timestamp_us(12_000_000_000, now, mode="header") == (12_000_000, True)
    assert choose_timestamp_us(now, now, mode="receipt", receipt_latency_ms=20) == (now // 1000 - 20_000, False)
