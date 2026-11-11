# Generated / assisted by Claude
"""ROS 2 node: score a simulated mapping run against ground truth, then exit 0 (pass) or 1 (fail).

Run it next to sim_mapping.launch.py (scripts/sim_e2e_test.sh does both):

  * FAST-LIO /Odometry is compared with /sim/ground_truth (position RMSE / max error, yaw RMSE);
  * every /fmu/in/vehicle_visual_odometry sample must be the FLU -> FRD image of a FAST-LIO pose
    (only checked when px4_msgs is available and the bridge runs with an identity mount);
  * /hummingbird/map/tile must deliver height tiles when the tiler is running;
  * the map written by /hummingbird/save_map must lie on the surfaces of the simulated room.
"""

from __future__ import annotations

import json
import math
import sys

import numpy as np


def yaw_of_xyzw(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def score_trajectory(est, truth):
    """est / truth: lists of (t, x, y, z, yaw). Truth is interpolated at the estimate's stamps.

    Returns a dict with n, rmse_m, max_m, yaw_rmse_deg (None values when nothing overlaps).
    """
    if len(est) < 2 or len(truth) < 2:
        return {"n": 0, "rmse_m": None, "max_m": None, "yaw_rmse_deg": None}
    e = np.asarray(sorted(est), dtype=float)
    g = np.asarray(sorted(truth), dtype=float)
    keep = (e[:, 0] >= g[0, 0]) & (e[:, 0] <= g[-1, 0])
    e = e[keep]
    if e.shape[0] == 0:
        return {"n": 0, "rmse_m": None, "max_m": None, "yaw_rmse_deg": None}
    gx, gy, gz = (np.interp(e[:, 0], g[:, 0], g[:, k]) for k in (1, 2, 3))
    gyaw = np.interp(e[:, 0], g[:, 0], np.unwrap(g[:, 4]))
    err = np.sqrt((e[:, 1] - gx) ** 2 + (e[:, 2] - gy) ** 2 + (e[:, 3] - gz) ** 2)
    dyaw = np.angle(np.exp(1j * (e[:, 4] - gyaw)))
    return {"n": int(e.shape[0]), "rmse_m": float(np.sqrt(np.mean(err ** 2))), "max_m": float(err.max()),
            "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(dyaw ** 2))))}


def score_saved_map(node, timeout_s: float = 30.0) -> dict:
    """Ask map_recorder to save, then measure how far the saved points are from the real room."""
    import rclpy
    from std_srvs.srv import Trigger

    from hummingbird_bringup import pcd_io
    from hummingbird_bringup.sim_world import Flight, distance_to_surfaces

    client = node.create_client(Trigger, "/hummingbird/save_map")
    if not client.wait_for_service(timeout_sec=5.0):
        return {"error": "/hummingbird/save_map is not available (map_recorder not running?)"}
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    res = future.result()
    if res is None or not res.success:
        return {"error": f"save_map failed: {None if res is None else res.message}"}
    path = res.message.split(" to ", 1)[-1].strip()
    try:
        cloud = pcd_io.read_pcd(path)
    except (OSError, ValueError) as exc:
        return {"error": f"cannot read {path}: {exc}"}
    if cloud.shape[0] < 1000:
        return {"error": f"{path} holds only {cloud.shape[0]} points"}
    # FAST-LIO's frame starts at the IMU's rest pose; the default flight starts level.
    world = np.column_stack([cloud["x"], cloud["y"], cloud["z"]]).astype(float) + np.asarray(Flight().start)
    d = distance_to_surfaces(world)
    return {"file": path, "points": int(cloud.shape[0]), "median_m": float(np.median(d)),
            "p99_m": float(np.percentile(d, 99)), "within_5cm": float((d < 0.05).mean())}


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import ParameterDescriptor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(args=args)
    node = Node("hummingbird_sim_check")
    # This is a command-line tool: accept "-p duration_s:=46" as well as "46.0".
    loose = ParameterDescriptor(dynamic_typing=True)
    duration = float(node.declare_parameter("duration_s", 45.0, loose).value)
    max_rmse = float(node.declare_parameter("max_rmse_m", 0.15, loose).value)
    max_yaw = float(node.declare_parameter("max_yaw_rmse_deg", 3.0, loose).value)
    min_samples = int(node.declare_parameter("min_samples", 150, loose).value)
    expect_bridge = bool(node.declare_parameter("expect_bridge", True).value)
    expect_tiles = bool(node.declare_parameter("expect_tiles", True).value)
    expect_map = bool(node.declare_parameter("expect_map", True).value)
    max_map_p99 = float(node.declare_parameter("max_map_p99_m", 0.10, loose).value)

    est, truth, tiles = [], [], set()
    bridge = {"n": 0, "max_err": 0.0, "resets": set(), "frames": set()}
    last_odom = {}

    def stamp_s(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_est(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        est.append((stamp_s(m.header), p.x, p.y, p.z, yaw_of_xyzw(q.x, q.y, q.z, q.w)))
        last_odom["frd"] = (p.x, -p.y, -p.z)

    def on_truth(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        truth.append((stamp_s(m.header), p.x, p.y, p.z, yaw_of_xyzw(q.x, q.y, q.z, q.w)))

    node.create_subscription(Odometry, "/Odometry", on_est, 50)
    node.create_subscription(Odometry, "/sim/ground_truth", on_truth, 50)

    try:
        from px4_msgs.msg import VehicleOdometry
    except ImportError:
        VehicleOdometry = None
    if VehicleOdometry is not None:
        def on_px4(m):
            bridge["n"] += 1
            bridge["resets"].add(int(m.reset_counter))
            bridge["frames"].add(int(m.pose_frame))
            if "frd" in last_odom:
                # The bridge forwards the newest FAST-LIO pose; allow one sample of motion (< 0.2 m).
                bridge["max_err"] = max(bridge["max_err"], math.dist(last_odom["frd"], tuple(m.position)))
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=20)
        node.create_subscription(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", on_px4, px4_qos)

    try:
        from hummingbird_interfaces.msg import MapTile
        node.create_subscription(MapTile, "/hummingbird/map/tile", lambda m: tiles.add(m.tile_id), 50)
    except ImportError:
        expect_tiles = False

    end = node.get_clock().now().nanoseconds + int(duration * 1e9)
    try:
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    s = score_trajectory(est, truth)
    failures = []
    map_report = score_saved_map(node) if expect_map else None
    if expect_map:
        if map_report.get("error"):
            failures.append(f"map check: {map_report['error']}")
        elif map_report["p99_m"] > max_map_p99:
            failures.append(f"99% of map points should be within {max_map_p99} m of a real surface, "
                            f"got {map_report['p99_m']:.3f} m")
    if s["n"] < min_samples:
        failures.append(f"only {s['n']} FAST-LIO poses overlap ground truth (need {min_samples})")
    else:
        if s["rmse_m"] > max_rmse:
            failures.append(f"position RMSE {s['rmse_m']:.3f} m > {max_rmse} m")
        if s["yaw_rmse_deg"] > max_yaw:
            failures.append(f"yaw RMSE {s['yaw_rmse_deg']:.2f} deg > {max_yaw} deg")
    if expect_bridge and VehicleOdometry is not None:
        if bridge["n"] < min_samples:
            failures.append(f"only {bridge['n']} VehicleOdometry samples reached the PX4 topic")
        if bridge["max_err"] > 0.2:
            failures.append(f"bridge output is {bridge['max_err']:.3f} m away from FLU->FRD of FAST-LIO")
        if bridge["resets"] - {0}:
            failures.append(f"bridge bumped reset_counter during a continuous flight: {sorted(bridge['resets'])}")
    if expect_tiles and not tiles:
        failures.append("no MapTile received on /hummingbird/map/tile")

    report = {
        "pass": not failures, "failures": failures, "fast_lio_vs_truth": s,
        "bridge": {"samples": bridge["n"], "max_err_m": round(bridge["max_err"], 4),
                   "reset_counters": sorted(bridge["resets"]), "pose_frames": sorted(bridge["frames"]),
                   "checked": VehicleOdometry is not None},
        "map_tiles": len(tiles),
        "saved_map": map_report,
    }
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
