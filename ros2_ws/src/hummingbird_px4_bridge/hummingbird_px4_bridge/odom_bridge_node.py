"""ROS 2 node: FAST-LIO2 /Odometry  ->  PX4 /fmu/in/vehicle_visual_odometry.

Safety scope (by design, enforced by tests/test_no_command_paths.py):
  * publishes ONLY px4_msgs/VehicleOdometry on the visual-odometry input topic;
  * never publishes vehicle-command, offboard-mode, trajectory-setpoint or actuator topics;
  * never arms, never changes flight mode.
Feeding the estimator still affects flight once PX4 fuses it: validate on the bench and in SITL.
"""

from __future__ import annotations

from hummingbird_px4_bridge.odom_conversion import BridgeConfig, OdometryConverter, choose_timestamp_us


def _declare(node, name, default):
    return node.declare_parameter(name, default).value


class OdomBridge:
    def __init__(self, node):
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleOdometry
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy

        self._node = node
        self._VehicleOdometry = VehicleOdometry
        in_topic = _declare(node, "input_topic", "/Odometry")
        out_topic = _declare(node, "output_topic", "/fmu/in/vehicle_visual_odometry")
        self._stamp_mode = _declare(node, "stamp_mode", "auto")
        self._max_skew_s = float(_declare(node, "max_header_skew_s", 0.5))
        self._receipt_latency_ms = float(_declare(node, "receipt_latency_ms", 0.0))
        self._max_rate_hz = float(_declare(node, "max_rate_hz", 50.0))

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
            quality=int(_declare(node, "quality", 0)),
        )
        self._conv = OdometryConverter(cfg)
        self._last_pub_ns = 0
        self._warned_receipt = False

        # PX4 subscribers are BEST_EFFORT/VOLATILE; a best-effort publisher matches them.
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = node.create_publisher(VehicleOdometry, out_topic, qos_out)
        self._sub = node.create_subscription(Odometry, in_topic, self._on_odom, 20)
        node.get_logger().info(
            f"px4 odom bridge: {in_topic} -> {out_topic} frame={cfg.output_frame} "
            f"cov={cfg.covariance_source} stamp={self._stamp_mode} (no arming / no commands)"
        )

    def _on_odom(self, msg):
        now_ns = self._node.get_clock().now().nanoseconds
        if self._max_rate_hz > 0 and now_ns - self._last_pub_ns < 1e9 / self._max_rate_hz:
            return
        hdr_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        t_us, used_header = choose_timestamp_us(
            hdr_ns, now_ns, self._stamp_mode, self._max_skew_s, self._receipt_latency_ms
        )
        if not used_header and self._stamp_mode == "auto" and not self._warned_receipt:
            self._node.get_logger().warn(
                "FAST-LIO header stamp is far from the ROS clock (LiDAR not time-synced?). "
                "Using receipt time; set up PTP for the Mid-360 and tune EKF2_EV_DELAY."
            )
            self._warned_receipt = True

        pose = msg.pose.pose
        out = self._conv.convert(
            t_us,
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            covariance=list(msg.pose.covariance),
            angular_velocity_flu=None,  # FAST_LIO_ROS2 does not fill twist
        )
        if out is None:
            self._node.get_logger().warn("dropping invalid FAST-LIO odometry sample")
            return

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


def main(args=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("hummingbird_px4_odom_bridge")
    OdomBridge(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
