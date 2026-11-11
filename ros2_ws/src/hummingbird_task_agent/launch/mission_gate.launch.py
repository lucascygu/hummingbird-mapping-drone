# Generated / assisted by Grok
# Extended by Claude: site limits come from config/mission_gate.yaml (override with gate_config:=...)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("hummingbird_task_agent"), "config", "mission_gate.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("gate_config", default_value=default_cfg,
                                  description="Site geofence / speed limits for mission_gate"),
            Node(
                package="hummingbird_task_agent",
                executable="mission_gate",
                name="hummingbird_mission_gate",
                output="screen",
                parameters=[LaunchConfiguration("gate_config")],
            ),
        ]
    )
