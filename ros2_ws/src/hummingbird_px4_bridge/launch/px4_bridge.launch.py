import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("hummingbird_px4_bridge"), "config", "px4_bridge.yaml"
    )
    cfg = LaunchConfiguration("bridge_config")
    return LaunchDescription([
        DeclareLaunchArgument("bridge_config", default_value=default_cfg),
        Node(
            package="hummingbird_px4_bridge",
            executable="odom_bridge",
            name="hummingbird_px4_odom_bridge",
            parameters=[cfg],
            output="screen",
        ),
    ])
