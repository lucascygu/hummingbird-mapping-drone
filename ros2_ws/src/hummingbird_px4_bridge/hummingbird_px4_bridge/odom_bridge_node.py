# Generated / assisted by Grok
# Reviewed and extended by Claude (sticky time source, discontinuity hold-off, health status)
"""ROS 2 node: FAST-LIO2 /Odometry  ->  PX4 /fmu/in/vehicle_visual_odometry.

Safety scope (by design, enforced by tests/test_no_command_paths.py):
  * publishes ONLY px4_msgs/VehicleOdometry on the visual-odometry input topic
    (plus a std_msgs/String health report on /hummingbird/px4_bridge/status);
  * never publishes vehicle-command, offboard-mode, trajectory-setpoint or actuator topics;
  * never arms, never changes flight mode.
Feeding the estimator still affects flight once PX4 fuses it: validate on the bench and in SITL.
"""

from __future__ import annotations

import json

from hummingbird_px4_bridge._runtime import init_ros, spin_until_stopped
from hummingbird_px4_bridge.odom_conversion import BridgeConfig, OdometryConverter, StampSelector


def _declare(node, name, default):
    return node.declare_parameter(name, default).value


class OdomBridge:
    def __init__(self, node):
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleOdometry
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String

        self._node = node
        self._VehicleOdometry = VehicleOdometry
        self._String = String
        in_topic = _declare(node, "input_topic", "/Odometry")
        out_topic = _declare(node, "output_topic", "/fmu/in/vehicle_visual_odometry")
        self._stamp = StampSelector(
            mode=_declare(node, "stamp_mode", "auto"),
            max_skew_s=float(_declare(node, "max_header_skew_s", 0.5)),
            receipt_latency_ms=float(_declare(node, "receipt_latency_ms", 0.0)),
        )
        self._max_rate_hz = float(_declare(node, "max_rate_hz", 50.0))
        self._stale_after_s = float(_declare(node, "stale_after_s", 1.0))

        cfg = BridgeConfig(
            output_frame=_declare(node, "output_frame", "frd"),
            imu_to_base_xyz=tuple(_declare(node, "imu_to_base_xyz", [0.0, 0.0, 0.0])),
            imu_to_base_rpy=tuple(_declare(node, "imu_to_base_rpy", [0.0, 0.0, 0.0])),
            reexpress_world_in_vehicle_frame=bool(_declare(node, "reexpress_world_in_vehicle_frame", True)),
            covariance_source=_declare(node, "covariance_source", "fixed"),
            fixed_position_variance=float(_declare(node, "fixed_position_variance", 0.01)),
            fixed_orientation_variance=float(_declare(node, "fixed_orientation_variance", 0.0025)),
            estimate_velocity=bool(_declare(node, "estimate_velocity", False)),
            velocity_lpf_alpha=float(_declare(node, "velocity_lpf_alpha", 0.5)),
            reset_jump_m=float(_declare(node, "reset_jump_m", 1.0)),
            max_plausible_speed_mps=float(_declare(node, "max_plausible_speed_mps", 30.0)),
            holdoff_s=float(_declare(node, "holdoff_s", 1.0)),
            gap_reset_s=float(_declare(node, "gap_reset_s", 1.0)),
            quality=int(_declare(node, "quality", 0)),
        )
        self._conv = OdometryConverter(cfg)
        self._last_pub_ns = 0
        self._last_rx_ns = 0
        self._rx_count = 0
        self._rx_count_at_status = 0
        self._stale_reported = False

        # PX4 subscribers are BEST_EFFORT/VOLATILE; a best-effort publisher matches them.
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = node.create_publisher(VehicleOdometry, out_topic, qos_out)
        self._status_pub = node.create_publisher(String, "/hummingbird/px4_bridge/status", 10)
        self._sub = node.create_subscription(Odometry, in_topic, self._on_odom, 20)
        self._status_period_s = 1.0
        self._status_timer = node.create_timer(self._status_period_s, self._on_status_timer)
        node.get_logger().info(
            f"px4 odom bridge: {in_topic} -> {out_topic} frame={cfg.output_frame} "
            f"cov={cfg.covariance_source} stamp={self._stamp.mode} (no arming / no commands)"
        )

    def _on_odom(self, msg):
        log = self._node.get_logger()
        now_ns = self._node.get_clock().now().nanoseconds
        self._last_rx_ns = now_ns
        self._rx_count += 1
        if self._stale_reported:
            log.info("FAST-LIO odometry is arriving again")
            self._stale_reported = False
        if self._max_rate_hz > 0 and now_ns - self._last_pub_ns < 1e9 / self._max_rate_hz:
            return
        hdr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t_us, used_header, changed = self._stamp.select(hdr_ns, now_ns)
        if changed and self._stamp.mode == "auto":
            if used_header:
                log.info("timestamps: using the FAST-LIO header stamp (LiDAR clock is in sync)")
            else:
                log.warn(
                    "FAST-LIO header stamp is far from the ROS clock (LiDAR not time-synced?). "
                    "Using receipt time; set up PTP for the Mid-360 and tune EKF2_EV_DELAY."
                )

        pose = msg.pose.pose
        out = self._conv.convert(
            t_us,
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            covariance=list(msg.pose.covariance),
            angular_velocity_flu=None,  # FAST_LIO_ROS2 does not fill twist
        )
        if out is None:
            reason = self._conv.last_drop_reason
            if reason == "holdoff":
                log.error(
                    "FAST-LIO pose jumped implausibly: NOT forwarding odometry to PX4 until the "
                    "stream has been steady again (check for divergence before flying on it)",
                    throttle_duration_sec=2.0,
                )
            else:
                log.warn(f"dropping FAST-LIO odometry sample ({reason})", throttle_duration_sec=2.0)
            return
        if self._conv.last_session_event:
            log.warn(
                f"new odometry session after '{self._conv.last_session_event}': "
                f"reset_counter={out.reset_counter} (PX4 re-aligns to external vision)"
            )

        m = self._VehicleOdometry()
        m.timestamp = int(now_ns // 1000)
        m.timestamp_sample = out.timestamp_sample_us
        m.pose_frame = out.pose_frame
        m.position = [float(c) for c in out.position]
        m.q = [float(c) for c in out.q]
        m.velocity_frame = out.velocity_frame
        m.velocity = [float(c) for c in out.velocity]
        m.angular_velocity = [float(c) for c in out.angular_velocity]
        m.position_variance = [float(c) for c in out.position_variance]
        m.orientation_variance = [float(c) for c in out.orientation_variance]
        m.velocity_variance = [float(c) for c in out.velocity_variance]
        m.reset_counter = out.reset_counter
        m.quality = out.quality
        self._pub.publish(m)
        self._last_pub_ns = now_ns

    def _on_status_timer(self):
        now_ns = self._node.get_clock().now().nanoseconds
        age_s = (now_ns - self._last_rx_ns) * 1e-9 if self._last_rx_ns else None
        stale = age_s is None or age_s > self._stale_after_s
        if stale and self._last_rx_ns and not self._stale_reported:
            self._node.get_logger().error(
                f"no FAST-LIO odometry for {age_s:.1f} s: PX4 is no longer receiving external vision"
            )
            self._stale_reported = True
        rate = (self._rx_count - self._rx_count_at_status) / self._status_period_s
        self._rx_count_at_status = self._rx_count
        if stale:
            state = "waiting" if not self._last_rx_ns else "stale"
        else:
            state = "holdoff" if self._conv.holding else "ok"
        msg = self._String()
        msg.data = json.dumps({
            "state": state,                       # waiting | ok | holdoff | stale
            "input_rate_hz": round(rate, 1),
            "input_age_s": None if age_s is None else round(age_s, 3),
            "time_source": {None: "none", True: "header", False: "receipt"}[self._stamp.use_header],
            "reset_counter": self._conv.reset_counter,
            "px4_subscribers": self._pub.get_subscription_count(),
            **self._conv.stats,
        })
        self._status_pub.publish(msg)


def main(args=None):
    from rclpy.node import Node

    init_ros(args)
    node = Node("hummingbird_px4_odom_bridge")
    OdomBridge(node)
    spin_until_stopped(node)


if __name__ == "__main__":
    main()
