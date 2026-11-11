# Generated / assisted by Claude
"""In-process ROS 2 test of the mission_gate node (services, GroundTask topic, published setpoints).

Skipped automatically when rclpy or the generated hummingbird_interfaces are not importable, so
scripts/run_tests.sh still works on a machine without ROS. Run it inside a built workspace:
    colcon test --packages-select hummingbird_task_agent
"""

import json
import math

import pytest

rclpy = pytest.importorskip("rclpy")
interfaces_msg = pytest.importorskip("hummingbird_interfaces.msg")
interfaces_srv = pytest.importorskip("hummingbird_interfaces.srv")

from geometry_msgs.msg import Pose  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from hummingbird_task_agent.mission_gate_node import MissionGateNode  # noqa: E402

GroundTask = interfaces_msg.GroundTask
HighLevelSetpoint = interfaces_msg.HighLevelSetpoint
ApproveMission = interfaces_srv.ApproveMission
RequestMission = interfaces_srv.RequestMission


def pose(x, y, z, yaw=None):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
    if yaw is not None:
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Harness:
    def __init__(self):
        self.gate_node = Node("hummingbird_mission_gate", parameter_overrides=[
            Parameter("geofence_radius_m", value=50.0),
            Parameter("geofence_max_altitude_m", value=20.0),
            Parameter("draft_ttl_s", value=60.0),
        ])
        self.gate = MissionGateNode(self.gate_node)
        self.client_node = Node("gate_test_client")
        self.setpoints, self.status = [], []
        self.client_node.create_subscription(
            HighLevelSetpoint, "/hummingbird/task/setpoint_request", self.setpoints.append, 300)
        self.client_node.create_subscription(
            String, "/hummingbird/status", lambda m: self.status.append(json.loads(m.data)), 50)
        self.request = self.client_node.create_client(RequestMission, "/hummingbird/mission/request")
        self.approve = self.client_node.create_client(ApproveMission, "/hummingbird/mission/approve")
        self.task_pub = self.client_node.create_publisher(GroundTask, "/hummingbird/ground/task", 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.gate_node)
        self.executor.add_node(self.client_node)
        assert self.request.wait_for_service(timeout_sec=10.0)
        assert self.approve.wait_for_service(timeout_sec=10.0)
        self.spin_until(lambda: self.task_pub.get_subscription_count() > 0
                        and self.gate_node.count_subscribers("/hummingbird/task/setpoint_request") > 0
                        and self.gate_node.count_subscribers("/hummingbird/status") > 0)

    def spin_until(self, cond, timeout=10.0):
        end = self.client_node.get_clock().now().nanoseconds + int(timeout * 1e9)
        while not cond():
            assert self.client_node.get_clock().now().nanoseconds < end, "timed out"
            self.executor.spin_once(timeout_sec=0.05)

    def call(self, client, req):
        fut = client.call_async(req)
        self.spin_until(fut.done)
        return fut.result()

    def close(self):
        self.executor.shutdown()
        self.gate_node.destroy_node()
        self.client_node.destroy_node()


@pytest.fixture()
def h():
    rclpy.init()
    harness = Harness()
    try:
        yield harness
    finally:
        harness.close()
        rclpy.shutdown()


def mission(**kw):
    r = RequestMission.Request()
    r.mission_id, r.mission_type, r.frame_id = "m1", "map_region", "map"
    r.waypoints = [pose(1, 1, 10), pose(2, 2, 10, yaw=math.pi / 2), pose(3, 3, 5)]
    r.arm_approval = r.takeoff_approval = True
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def test_service_releases_ordered_setpoints(h):
    res = h.call(h.request, mission(max_speed_mps=2.0))
    assert res.accepted, res.reason
    h.spin_until(lambda: len(h.setpoints) == 3 and h.status)
    assert [m.sequence for m in h.setpoints] == [0, 1, 2]
    assert all(m.total == 3 and m.mission_id == "m1" and m.frame_id == "map" for m in h.setpoints)
    assert all(m.action == HighLevelSetpoint.ACTION_GOTO and m.max_speed_mps == 2.0 for m in h.setpoints)
    assert [m.yaw_valid for m in h.setpoints] == [False, True, False]
    q = h.setpoints[1].pose.orientation
    assert 2 * math.atan2(q.z, q.w) == pytest.approx(math.pi / 2)
    assert h.setpoints[2].pose.position.z == 5.0
    assert h.status[-1]["event"] == "accepted" and h.status[-1]["setpoints"] == 3


def test_service_refuses_without_approval_and_outside_site_geofence(h):
    res = h.call(h.request, mission(arm_approval=False))
    assert not res.accepted and "arm_approval" in res.reason
    # node parameter geofence_radius_m=50; the request tries to widen it to 10 km
    res = h.call(h.request, mission(waypoints=[pose(80, 0, 10)], geofence_radius_m=10_000.0))
    assert not res.accepted and "geofence" in res.reason
    h.spin_until(lambda: len(h.status) == 2)
    assert [s["code"] for s in h.status] == ["needs_arm_approval", "outside_geofence"]
    assert h.setpoints == []


def test_rtl_is_never_blocked(h):
    r = RequestMission.Request()
    r.mission_type = "rtl"            # no id, no frame, no approval
    res = h.call(h.request, r)
    assert res.accepted and res.mission_id == "safety-rtl"
    h.spin_until(lambda: len(h.setpoints) == 1)
    assert h.setpoints[0].action == HighLevelSetpoint.ACTION_RTL
    assert (h.setpoints[0].sequence, h.setpoints[0].total) == (0, 1)


def test_ground_task_waits_for_human_approval(h):
    task = GroundTask()
    task.task_id, task.task_type, task.frame_id = "g1", "inspect", "map"
    task.waypoints = [pose(5, 5, 8)]
    h.task_pub.publish(task)
    h.spin_until(lambda: h.status)
    assert h.status[-1]["event"] == "pending" and h.status[-1]["code"] == "pending_approval"
    assert h.setpoints == []          # nothing moves before a human approves

    a = ApproveMission.Request()
    a.mission_id, a.arm_approval, a.takeoff_approval = "g1", True, False
    res = h.call(h.approve, a)
    assert not res.accepted and "takeoff" in res.reason
    a.takeoff_approval = True
    res = h.call(h.approve, a)
    assert res.accepted
    h.spin_until(lambda: len(h.setpoints) == 1)
    assert h.setpoints[0].mission_id == "g1" and h.setpoints[0].mission_type == "inspect"
    assert not h.call(h.approve, a).accepted    # the approval was consumed


def test_ground_task_land_is_released_immediately(h):
    task = GroundTask()
    task.task_id, task.task_type = "g2", "land"
    h.task_pub.publish(task)
    h.spin_until(lambda: len(h.setpoints) == 1)
    assert h.setpoints[0].action == HighLevelSetpoint.ACTION_LAND
