from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hummingbird_task_agent",
                executable="mission_gate",
                name="hummingbird_mission_gate",
                output="screen",
                parameters=[{"allowed_frames": ["hummingbird_map", "map", "odom"]}],
            )
        ]
    )
