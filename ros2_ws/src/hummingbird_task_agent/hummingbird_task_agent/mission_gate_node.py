# Generated / assisted by Grok
"""ROS 2 node wrapper around the pure mission gate.

Publishes only HighLevelSetpoint messages. It does not call PX4 arm/takeoff
APIs and does not publish actuator topics. Not tested on hardware.
"""

from __future__ import annotations

from hummingbird_task_agent.mission_gate import MissionRequest, Waypoint, evaluate_mission


def _pose_to_waypoint(pose) -> Waypoint:
    return Waypoint(x=pose.position.x, y=pose.position.y, z=pose.position.z)


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
    )


class MissionGateNode:
    """Constructed inside main() after rclpy is imported."""

    def __init__(self, node):
        from hummingbird_interfaces.msg import HighLevelSetpoint
        from hummingbird_interfaces.srv import RequestMission
        from geometry_msgs.msg import Pose
        from std_msgs.msg import String

        self._node = node
        self._HighLevelSetpoint = HighLevelSetpoint
        self._Pose = Pose
        self._allowed = node.declare_parameter(
            "allowed_frames", ["hummingbird_map", "map", "odom"]
        ).value
        self._setpoint_pub = node.create_publisher(
            HighLevelSetpoint, "/hummingbird/task/setpoint_request", 10
        )
        self._status_pub = node.create_publisher(String, "/hummingbird/status", 10)
        self._srv = node.create_service(
            RequestMission, "/hummingbird/mission/request", self._on_request
        )
        node.get_logger().info(
            "mission_gate ready: will publish high-level setpoints only; "
            "arming without arm_approval is refused"
        )

    def _on_request(self, request, response):
        from std_msgs.msg import String

        result = evaluate_mission(
            build_request_from_srv(request),
            allowed_frames=list(self._allowed) if self._allowed else None,
        )
        response.accepted = result.accepted
        response.reason = result.reason
        response.mission_id = result.mission_id

        status = String()
        status.data = f"accepted={result.accepted} mission_id={result.mission_id} reason={result.reason}"
        self._status_pub.publish(status)

        if not result.accepted:
            self._node.get_logger().warn(result.reason)
            return response

        for wp in result.setpoints:
            msg = self._HighLevelSetpoint()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = request.frame_id
            msg.mission_id = result.mission_id
            msg.frame_id = request.frame_id
            msg.pose.position.x = wp.x
            msg.pose.position.y = wp.y
            msg.pose.position.z = wp.z
            msg.max_speed_mps = 0.0
            msg.yaw_valid = False
            self._setpoint_pub.publish(msg)

        self._node.get_logger().info(
            f"accepted mission {result.mission_id}: published {len(result.setpoints)} setpoint requests"
        )
        return response


def main(args=None):
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=args)
    node = Node("hummingbird_mission_gate")
    MissionGateNode(node)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
