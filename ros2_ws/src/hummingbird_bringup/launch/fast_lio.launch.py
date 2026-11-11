"""FAST-LIO2 mapping node with Hummingbird's Mid-360 config (no RViz; see mapping.launch.py)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hummingbird_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("fast_lio_config",
                              default_value=os.path.join(share, "config", "fast_lio_mid360.yaml")),
        DeclareLaunchArgument("map_dir", default_value="/tmp",
                              description="Directory for FAST-LIO's /map_save output"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_mapping",
            output="screen",
            parameters=[
                LaunchConfiguration("fast_lio_config"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map_file_path": PathJoinSubstitution(
                        [LaunchConfiguration("map_dir"), "fastlio_map.pcd"]),
                },
            ],
        ),
    ])
