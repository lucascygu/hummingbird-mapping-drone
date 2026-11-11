# Generated / assisted by Claude
"""The real mapping stack fed by a fake Mid-360 (no hardware needed).

  hummingbird_bringup/mid360_sim -> FAST-LIO2 -> PX4 bridge + map_recorder + map_tiler

  ros2 launch hummingbird_bringup sim_mapping.launch.py
  ros2 launch hummingbird_bringup sim_mapping.launch.py lidar_clock:=boot     # unsynced LiDAR clock
  ros2 run hummingbird_bringup sim_check                                      # score it (or scripts/sim_e2e_test.sh)

There is no PX4 here: the bridge publishes /fmu/in/vehicle_visual_odometry into the void, which
is enough to check the conversion. Nothing can arm or command anything.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("hummingbird_bringup")
    L = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", default_value="/tmp/hummingbird_sim_maps"),
        DeclareLaunchArgument("lidar_clock", default_value="ros",
                              description="ros = PTP-synced LiDAR stamps, boot = unsynced (since power-on)"),
        DeclareLaunchArgument("lap_s", default_value="30.0", description="Duration of the flown lap"),
        DeclareLaunchArgument("loop", default_value="false", description="Repeat the flight forever"),
        DeclareLaunchArgument("points_per_scan", default_value="6000"),
        DeclareLaunchArgument("with_mission_gate", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "mapping.launch.py")),
            launch_arguments={
                "with_livox": "false",
                "with_map_tiler": "true",
                "map_dir": L("map_dir"),
                "map_autosave_s": "10.0",
                "with_mission_gate": L("with_mission_gate"),
                "rviz": L("rviz"),
            }.items(),
        ),
        Node(
            package="hummingbird_bringup", executable="mid360_sim", name="hummingbird_mid360_sim",
            output="screen",
            parameters=[{
                "lidar_clock": L("lidar_clock"),
                "lap_s": ParameterValue(L("lap_s"), value_type=float),
                "loop": ParameterValue(L("loop"), value_type=bool),
                "points_per_scan": ParameterValue(L("points_per_scan"), value_type=int),
            }],
        ),
    ])
