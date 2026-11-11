# Generated / assisted by Claude
"""ROS 2 node: a fake Livox Mid-360 flying a known lap through a known room.

Publishes exactly what livox_ros_driver2 publishes with xfer_format=1, so the real mapping stack
can be run and checked on any machine, with no LiDAR, no flight controller and no airframe:

  /livox/lidar        livox_ros_driver2/CustomMsg   10 Hz, points in the LiDAR frame, per-point times
  /livox/imu          sensor_msgs/Imu               200 Hz, acceleration in g (as the Mid-360 reports it)
  /sim/ground_truth   nav_msgs/Odometry             pose FAST-LIO should report (camera_init -> body)

  ros2 launch hummingbird_bringup sim_mapping.launch.py

This is a test signal generator, not a flight simulator: there is no vehicle dynamics and no PX4.
"""

from __future__ import annotations

import numpy as np

from hummingbird_bringup._runtime import init_ros, spin_until_stopped
from hummingbird_bringup.sim_world import G, Flight, ScanSimulator, quat_xyzw_from_matrix


def main(args=None):
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    init_ros(args)
    node = Node("hummingbird_mid360_sim")
    imu_rate = float(node.declare_parameter("imu_rate_hz", 200.0).value)
    scan_period = 1.0 / float(node.declare_parameter("scan_rate_hz", 10.0).value)
    # "ros": stamps follow the ROS clock (a PTP-synced LiDAR). "boot": stamps count from LiDAR
    # power-on, which is what an unsynced Mid-360 does and what makes the bridge use receipt time.
    lidar_clock = node.declare_parameter("lidar_clock", "ros").value
    acc_in_g = bool(node.declare_parameter("imu_acc_in_g", True).value)
    acc_noise = float(node.declare_parameter("imu_acc_noise", 0.02).value)      # m/s^2
    gyro_noise = float(node.declare_parameter("imu_gyro_noise", 0.002).value)   # rad/s
    flight = Flight(
        t_rest=float(node.declare_parameter("rest_s", 3.0).value),
        t_lap=float(node.declare_parameter("lap_s", 30.0).value),
        start_roll=float(node.declare_parameter("start_roll", 0.0).value),
        start_pitch=float(node.declare_parameter("start_pitch", 0.0).value),
        loop=bool(node.declare_parameter("loop", False).value),
    )
    sim = ScanSimulator(
        flight=flight,
        points_per_scan=int(node.declare_parameter("points_per_scan", 6000).value),
        scan_period=scan_period,
        range_noise=float(node.declare_parameter("range_noise", 0.02).value),
        seed=int(node.declare_parameter("seed", 1).value),
    )
    rng = np.random.default_rng(sim.seed + 1)

    lidar_pub = node.create_publisher(CustomMsg, "/livox/lidar", 10)
    imu_pub = node.create_publisher(Imu, "/livox/imu", 400)
    truth_pub = node.create_publisher(Odometry, "/sim/ground_truth", 20)

    t_start_ns = node.get_clock().now().nanoseconds
    # Stamp of sim time 0 on the simulated LiDAR clock.
    stamp0_ns = t_start_ns if lidar_clock == "ros" else 100_000_000_000
    state = {"imu_k": 0, "scan_k": 0}
    imu_dt = 1.0 / imu_rate

    def stamp(t_sim: float):
        ns = stamp0_ns + int(round(t_sim * 1e9))
        return ns, divmod(ns, 1_000_000_000)

    def publish_imu(t_sim: float):
        f_b, w_b = flight.imu(t_sim)
        f_b = f_b + rng.normal(0.0, acc_noise, 3)
        w_b = w_b + rng.normal(0.0, gyro_noise, 3)
        if acc_in_g:
            f_b = f_b / G
        m = Imu()
        _, (m.header.stamp.sec, m.header.stamp.nanosec) = stamp(t_sim)
        m.header.frame_id = "livox_frame"
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = (float(c) for c in f_b)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (float(c) for c in w_b)
        imu_pub.publish(m)

    def publish_scan(t0: float):
        offset, xyz = sim.scan(t0)
        m = CustomMsg()
        ns, (m.header.stamp.sec, m.header.stamp.nanosec) = stamp(t0)
        m.header.frame_id = "livox_frame"
        m.timebase = ns
        m.point_num = int(xyz.shape[0])
        m.lidar_id = 0
        off_ns = np.rint(offset * 1e9).astype(np.int64)
        refl = np.clip(40.0 + 8.0 * xyz[:, 2], 1, 255).astype(np.int64)
        pts = []
        for i in range(xyz.shape[0]):
            p = CustomPoint()
            p.offset_time = int(off_ns[i])
            p.x, p.y, p.z = float(xyz[i, 0]), float(xyz[i, 1]), float(xyz[i, 2])
            p.reflectivity = int(refl[i])
            p.tag = 0
            p.line = i % 4
            pts.append(p)
        m.points = pts
        lidar_pub.publish(m)

        # Ground truth at the end of the frame, which is the instant FAST-LIO's odometry refers to.
        t_end = t0 + scan_period
        p_rel, r_rel = flight.relative_pose(t_end)
        o = Odometry()
        _, (o.header.stamp.sec, o.header.stamp.nanosec) = stamp(t_end)
        o.header.frame_id, o.child_frame_id = "camera_init", "body"
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = (float(c) for c in p_rel)
        q = o.pose.pose.orientation
        q.x, q.y, q.z, q.w = (float(c) for c in quat_xyzw_from_matrix(r_rel))
        truth_pub.publish(o)

    def on_timer():
        # Catch up to the wall clock: publish every IMU sample and every finished frame that is due.
        t_now = (node.get_clock().now().nanoseconds - t_start_ns) * 1e-9
        while state["imu_k"] * imu_dt <= t_now:
            publish_imu(state["imu_k"] * imu_dt)
            state["imu_k"] += 1
        while (state["scan_k"] + 1) * scan_period <= t_now:
            publish_scan(state["scan_k"] * scan_period)
            state["scan_k"] += 1

    node.create_timer(imu_dt, on_timer)
    node.get_logger().info(
        f"mid360_sim: {sim.points_per_scan} points/scan at {1.0 / scan_period:g} Hz, IMU {imu_rate:g} Hz, "
        f"flight of {flight.duration:g} s (rest {flight.t_rest:g} s first), lidar_clock={lidar_clock}"
    )
    spin_until_stopped(node)


if __name__ == "__main__":
    main()
