"""Livox Mid-360 driver (livox_ros_driver2) with Hummingbird's network config.

Mirrors livox_ros_driver2/launch_ROS2/msg_MID360_launch.py (xfer_format=1 = livox CustomMsg,
required by FAST-LIO's lidar_type=1) but points user_config_path at our MID360_config.json.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hummingbird_bringup")
    default_cfg = os.path.join(share, "config", "MID360_config.json")
    return LaunchDescription([
        DeclareLaunchArgument("livox_config", default_value=default_cfg,
                              description="Path to MID360_config.json (host / LiDAR IPs)"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,        # 1 = livox_ros_driver2/CustomMsg (FAST-LIO lidar_type 1)
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": LaunchConfiguration("publish_freq"),
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "user_config_path": LaunchConfiguration("livox_config"),
                "cmdline_input_bd_code": "livox0000000001",
            }],
        ),
    ])
