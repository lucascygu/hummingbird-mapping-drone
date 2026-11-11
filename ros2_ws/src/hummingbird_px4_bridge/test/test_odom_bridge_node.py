# Generated / assisted by Claude
"""In-process ROS 2 test of the odometry bridge node against the real px4_msgs definitions.

Skipped automatically when rclpy or px4_msgs are not importable, so scripts/run_tests.sh still
works on a machine without ROS. Run it inside a built workspace:
    colcon test --packages-select hummingbird_px4_bridge
"""

import json
import math
import struct

import pytest

rclpy = pytest.importorskip("rclpy")
px4_msgs = pytest.importorskip("px4_msgs.msg")

from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from hummingbird_px4_bridge.odom_bridge_node import OdomBridge  # noqa: E402

VehicleOdometry = px4_msgs.VehicleOdometry

# What PX4's uXRCE-DDS client uses for its data readers (uxrce_dds_client/utilities.hpp).
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=10)


class Harness:
    def __init__(self, **params):
        overrides = [Parameter(k, value=v) for k, v in params.items()]
        self.bridge_node = Node("hummingbird_px4_odom_bridge", parameter_overrides=overrides)
        self.bridge = OdomBridge(self.bridge_node)
        self.io = Node("bridge_test_io")
        self.out, self.status = [], []
        self.io.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry",
                                    self.out.append, PX4_QOS)
        self.io.create_subscription(String, "/hummingbird/px4_bridge/status",
                                    lambda m: self.status.append(json.loads(m.data)), 10)
        self.pub = self.io.create_publisher(Odometry, "/Odometry", 20)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.bridge_node)
        self.executor.add_node(self.io)
        self.spin_until(lambda: self.pub.get_subscription_count() > 0
                        and self.bridge_node.count_subscribers("/fmu/in/vehicle_visual_odometry") > 0)

    def spin_until(self, cond, timeout=10.0):
        end = self.io.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not cond():
            assert self.io.get_clock().now().nanoseconds < end, "timed out"
            self.executor.spin_once(timeout_sec=0.02)

    def spin_for(self, seconds):
        end = self.io.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.io.get_clock().now().nanoseconds < end:
            self.executor.spin_once(timeout_sec=0.02)

    def send(self, x, y, z, q_xyzw=(0.0, 0.0, 0.0, 1.0), stamp_ns=None):
        m = Odometry()
        now = self.io.get_clock().now()
        if stamp_ns is None:
            m.header.stamp = now.to_msg()
        else:
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(int(stamp_ns), 1_000_000_000)
        m.header.frame_id, m.child_frame_id = "camera_init", "body"
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = float(x), float(y), float(z)
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = (float(c) for c in q_xyzw)
        self.pub.publish(m)
        return now.nanoseconds

    def send_and_wait(self, *args, **kw):
        n = len(self.out)
        t = self.send(*args, **kw)
        self.spin_until(lambda: len(self.out) > n)
        return t, self.out[-1]

    def close(self):
        self.executor.shutdown()
        self.bridge_node.destroy_node()
        self.io.destroy_node()


@pytest.fixture()
def make():
    rclpy.init()
    made = []

    def factory(**params):
        made.append(Harness(**params))
        return made[-1]

    try:
        yield factory
    finally:
        for h in made:
            h.close()
        rclpy.shutdown()


def f32_norm(q):
    return math.sqrt(sum(struct.unpack("f", struct.pack("f", c))[0] ** 2 for c in q))


def test_flu_odometry_reaches_px4_topic_as_frd(make):
    h = make(max_rate_hz=0.0)
    yaw_left_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))        # ROS: +90 deg about z = left turn
    t_ns, m = h.send_and_wait(1.0, 2.0, 3.0, yaw_left_90)
    assert m.pose_frame == VehicleOdometry.POSE_FRAME_FRD
    assert list(m.position) == pytest.approx([1.0, -2.0, -3.0])     # forward, right, down
    w, x, y, z = m.q
    assert math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) == pytest.approx(-90.0, abs=1e-3)
    assert abs(1.0 - f32_norm(m.q)) <= 1e-5                         # EKF2 rejects anything looser
    assert m.velocity_frame == VehicleOdometry.VELOCITY_FRAME_UNKNOWN
    assert all(math.isnan(v) for v in m.velocity) and all(math.isnan(v) for v in m.angular_velocity)
    assert list(m.position_variance) == pytest.approx([0.01] * 3)
    assert list(m.orientation_variance) == pytest.approx([0.0025] * 3)
    assert m.reset_counter == 0 and m.quality == 0
    # header stamp is in sync with the ROS clock -> used as the sample time, in microseconds
    assert m.timestamp_sample == t_ns // 1000
    assert 0 <= m.timestamp - m.timestamp_sample < 2_000_000


def test_unsynced_lidar_clock_falls_back_to_receipt_time(make):
    h = make(max_rate_hz=0.0)
    before = h.io.get_clock().now().nanoseconds // 1000
    _, m = h.send_and_wait(0.0, 0.0, 0.0, stamp_ns=12_000_000_000)  # 12 s since LiDAR power-on
    after = h.io.get_clock().now().nanoseconds // 1000
    assert before <= m.timestamp_sample <= after


def test_divergence_is_not_forwarded_and_status_reports_it(make):
    h = make(max_rate_hz=0.0, holdoff_s=0.5, stale_after_s=5.0)
    h.send_and_wait(0.0, 0.0, 0.0)
    h.spin_for(0.05)
    h.send_and_wait(0.02, 0.0, 0.0)
    h.spin_for(0.05)
    n = len(h.out)
    h.send(80.0, 0.0, 0.0)                                           # 80 m in 50 ms
    h.spin_for(0.2)
    assert len(h.out) == n                                           # PX4 saw nothing of it
    h.spin_until(lambda: any(s["state"] == "holdoff" for s in h.status), timeout=5.0)
    h.spin_for(0.5)
    _, m = h.send_and_wait(80.1, 0.0, 0.0)                           # steady again after the hold-off
    assert m.reset_counter == 1
    h.spin_until(lambda: h.status and h.status[-1]["state"] == "ok" and h.status[-1]["glitches"] == 1,
                 timeout=5.0)
    assert h.status[-1]["px4_subscribers"] == 1 and h.status[-1]["time_source"] == "header"


def test_status_goes_stale_when_fast_lio_stops(make):
    h = make(max_rate_hz=0.0, stale_after_s=0.3)
    h.spin_until(lambda: h.status, timeout=5.0)
    assert h.status[-1]["state"] == "waiting"
    h.send_and_wait(0.0, 0.0, 0.0)
    h.spin_until(lambda: h.status[-1]["state"] == "stale", timeout=5.0)
