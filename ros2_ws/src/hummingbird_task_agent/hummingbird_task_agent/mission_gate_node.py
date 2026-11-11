# Generated / assisted by Grok
# Reviewed and extended by Claude (GroundTask drafts, approval service, site geofence parameters,
# ordered setpoints, machine-readable status)
"""ROS 2 node wrapper around the pure mission gate.

Publishes only HighLevelSetpoint messages. It does not call PX4 arm/takeoff
APIs and does not publish actuator topics. Not tested on hardware.

Interfaces
  service  /hummingbird/mission/request   RequestMission   one-shot: approval travels with the request
  service  /hummingbird/mission/approve   ApproveMission   human approval for a parked GroundTask draft
  sub      /hummingbird/ground/task       GroundTask       parked as a draft (rtl / land: released at once)
  pub      /hummingbird/task/setpoint_request  HighLevelSetpoint  ordered, sequence / total filled
  pub      /hummingbird/status            std_msgs/String  one JSON object per gate decision

The approval flags are a procedural gate, not a security boundary: ROS 2 without SROS2 lets any
node on the domain call these services. The hard barrier is that nothing in this workspace can
arm the vehicle; arming stays with the pilot (RC / QGroundControl).
"""

from __future__ import annotations

import json
import time

from hummingbird_task_agent._runtime import init_ros, spin_until_stopped
from hummingbird_task_agent.mission_flow import (
    MissionFlow, expand_setpoints, quaternion_from_yaw, yaw_from_quaternion,
)
from hummingbird_task_agent.mission_gate import GateConfig, MissionRequest, Waypoint

NAN = float("nan")


def _pose_to_waypoint(pose) -> Waypoint:
    o = pose.orientation
    try:
        yaw = yaw_from_quaternion(o.x, o.y, o.z, o.w)
    except ValueError:
        yaw = NAN  # the gate refuses a non-finite yaw with an explicit reason
    return Waypoint(x=pose.position.x, y=pose.position.y, z=pose.position.z, yaw=yaw)


def build_request_from_srv(request) -> MissionRequest:
    return MissionRequest(
        mission_id=request.mission_id,
        mission_type=request.mission_type,
        frame_id=request.frame_id,
        waypoints=tuple(_pose_to_waypoint(p) for p in request.waypoints),
        arm_approval=bool(request.arm_approval),
        takeoff_approval=bool(request.takeoff_approval),
        geofence_max_altitude_m=float(request.geofence_max_altitude_m),
        geofence_radius_m=float(request.geofence_radius_m),
        max_speed_mps=float(request.max_speed_mps),
    )


def build_request_from_task(task) -> MissionRequest:
    """GroundTask -> MissionRequest. A GroundTask can never carry approval."""
    return MissionRequest(
        mission_id=task.task_id,
        mission_type=task.task_type,
        frame_id=task.frame_id,
        waypoints=tuple(_pose_to_waypoint(p) for p in task.waypoints),
        arm_approval=False,
        takeoff_approval=False,
        geofence_max_altitude_m=float(task.geofence_max_altitude_m),
        geofence_radius_m=float(task.geofence_radius_m),
        max_speed_mps=float(task.max_speed_mps),
    )


def status_json(event: str, source: str, result, setpoints: int = 0) -> str:
    return json.dumps({
        "event": event,              # accepted | refused | pending | expired
        "source": source,            # service | ground_task | approve | timer
        "mission_id": result.mission_id,
        "mission_type": result.mission_type,
        "code": result.code,
        "reason": result.reason,
        "setpoints": setpoints,
    }, ensure_ascii=False)


class MissionGateNode:
    """Constructed inside main() after rclpy is imported."""

    def __init__(self, node):
        from hummingbird_interfaces.msg import GroundTask, HighLevelSetpoint
        from hummingbird_interfaces.srv import ApproveMission, RequestMission
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._node = node
        self._HighLevelSetpoint = HighLevelSetpoint
        self._String = String

        allowed = node.declare_parameter(
            "allowed_frames", ["hummingbird_map", "map", "odom"]
        ).value
        allowed = tuple(f for f in (allowed or []) if f)
        # Site limits. PLACEHOLDERS: replace with the surveyed test-site values before any flight.
        config = GateConfig(
            max_altitude_m=float(node.declare_parameter("geofence_max_altitude_m", 30.0).value),
            radius_m=float(node.declare_parameter("geofence_radius_m", 100.0).value),
            home=tuple(float(c) for c in node.declare_parameter("home_xyz", [0.0, 0.0, 0.0]).value),
            allowed_frames=allowed or None,
            max_waypoints=int(node.declare_parameter("max_waypoints", 200).value),
            max_speed_limit_mps=float(node.declare_parameter("max_speed_limit_mps", 5.0).value),
        )
        self._flow = MissionFlow(
            config,
            draft_ttl_s=float(node.declare_parameter("draft_ttl_s", 120.0).value),
            max_drafts=int(node.declare_parameter("max_drafts", 16).value),
        )

        # VOLATILE on purpose: a consumer that starts late must not replay an old approval.
        # Depth covers one whole mission so an ordered burst is never truncated.
        setpoint_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, config.max_waypoints),
        )
        self._setpoint_pub = node.create_publisher(
            HighLevelSetpoint, "/hummingbird/task/setpoint_request", setpoint_qos
        )
        self._status_pub = node.create_publisher(String, "/hummingbird/status", 10)
        self._srv = node.create_service(
            RequestMission, "/hummingbird/mission/request", self._on_request
        )
        self._approve_srv = node.create_service(
            ApproveMission, "/hummingbird/mission/approve", self._on_approve
        )
        self._task_sub = node.create_subscription(
            GroundTask, "/hummingbird/ground/task", self._on_ground_task, 10
        )
        self._expire_timer = node.create_timer(1.0, self._on_expire_timer)
        node.get_logger().info(
            "mission_gate ready: will publish high-level setpoints only; "
            "arming without arm_approval is refused; rtl / land are never blocked. "
            f"site geofence: altitude <= {config.max_altitude_m:g} m, radius <= {config.radius_m:g} m "
            f"around home {config.home}, speed <= {config.max_speed_limit_mps:g} m/s"
        )

    # ------------------------------------------------------------------ helpers
    def _publish_status(self, event: str, source: str, result, setpoints: int = 0) -> None:
        msg = self._String()
        msg.data = status_json(event, source, result, setpoints)
        self._status_pub.publish(msg)

    def _release(self, result, source: str) -> int:
        """Publish the ordered setpoints of an accepted mission. Returns how many were sent."""
        specs = expand_setpoints(result)
        stamp = self._node.get_clock().now().to_msg()
        for spec in specs:
            msg = self._HighLevelSetpoint()
            msg.header.stamp = stamp
            msg.header.frame_id = spec.frame_id
            msg.mission_id = spec.mission_id
            msg.mission_type = spec.mission_type
            msg.frame_id = spec.frame_id
            msg.action = spec.action
            msg.sequence = spec.sequence
            msg.total = spec.total
            msg.pose.position.x = spec.x
            msg.pose.position.y = spec.y
            msg.pose.position.z = spec.z
            if spec.yaw_valid:
                qx, qy, qz, qw = quaternion_from_yaw(spec.yaw)
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw
            msg.max_speed_mps = spec.max_speed_mps
            msg.yaw_valid = spec.yaw_valid
            self._setpoint_pub.publish(msg)
        self._publish_status("accepted", source, result, len(specs))
        self._node.get_logger().info(
            f"accepted {result.mission_type} mission {result.mission_id!r} via {source}: "
            f"published {len(specs)} setpoint request(s)"
        )
        return len(specs)

    def _refuse(self, result, source: str) -> None:
        self._publish_status("refused", source, result)
        self._node.get_logger().warn(
            f"refused mission {result.mission_id!r} via {source}: {result.reason}"
        )

    # ------------------------------------------------------------------ callbacks
    def _on_request(self, request, response):
        result = self._flow.submit(build_request_from_srv(request))
        response.accepted = result.accepted
        response.reason = result.reason
        response.mission_id = result.mission_id
        if result.accepted:
            self._release(result, "service")
        else:
            self._refuse(result, "service")
        return response

    def _on_ground_task(self, task) -> None:
        outcome = self._flow.submit_draft(build_request_from_task(task), time.monotonic())
        if outcome.state == "released":
            self._release(outcome.result, "ground_task")
        elif outcome.state == "pending":
            self._publish_status("pending", "ground_task", outcome.result)
            self._node.get_logger().info(
                f"ground task {outcome.result.mission_id!r} parked: {outcome.result.reason}"
            )
        else:
            self._refuse(outcome.result, "ground_task")

    def _on_approve(self, request, response):
        result = self._flow.approve(
            request.mission_id, bool(request.arm_approval), bool(request.takeoff_approval),
            time.monotonic(),
        )
        response.accepted = result.accepted
        response.reason = result.reason
        response.mission_id = result.mission_id
        if result.accepted:
            self._release(result, "approve")
        else:
            self._refuse(result, "approve")
        return response

    def _on_expire_timer(self) -> None:
        from hummingbird_task_agent.mission_gate import GateResult

        for mission_id in self._flow.expire(time.monotonic()):
            expired = GateResult(False, "draft expired without approval", mission_id, (), "expired")
            self._publish_status("expired", "timer", expired)
            self._node.get_logger().warn(f"draft {mission_id!r} expired without approval")


def main(args=None):
    from rclpy.node import Node

    init_ros(args)
    node = Node("hummingbird_mission_gate")
    MissionGateNode(node)   # raises on an invalid site configuration: fail closed, do not spin
    spin_until_stopped(node)


if __name__ == "__main__":
    main()
