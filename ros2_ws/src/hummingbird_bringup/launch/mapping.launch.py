"""Full Hummingbird mapping stack on the Jetson.

  Livox Mid-360 -> livox_ros_driver2 (/livox/lidar CustomMsg, /livox/imu)
               -> FAST-LIO2 (/Odometry, /cloud_registered, TF camera_init->body)
               -> hummingbird_px4_bridge (/fmu/in/vehicle_visual_odometry, estimator input only)
               -> map_recorder (/hummingbird/save_map)
  + static TFs, optional MicroXRCEAgent, optional RViz, optional mission_gate.

Nothing in this launch arms the vehicle, switches modes or sends setpoints to PX4.

  ros2 launch hummingbird_bringup mapping.launch.py
  ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hummingbird_bringup")
    L = LaunchConfiguration

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
            launch_arguments=args.items(),
        )

    args = [
        DeclareLaunchArgument("livox_config", default_value=os.path.join(share, "config", "MID360_config.json")),
        DeclareLaunchArgument("fast_lio_config", default_value=os.path.join(share, "config", "fast_lio_mid360.yaml")),
        DeclareLaunchArgument("tf_config", default_value=os.path.join(share, "config", "static_tf.yaml")),
        DeclareLaunchArgument("map_dir", default_value=os.path.expanduser("~/hummingbird_maps")),
        DeclareLaunchArgument("map_voxel", default_value="0.05"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("with_livox", default_value="true", description="false when replaying a bag"),
        DeclareLaunchArgument("with_px4_bridge", default_value="true"),
        DeclareLaunchArgument("bridge_config", default_value=os.path.join(
            get_package_share_directory("hummingbird_px4_bridge"), "config", "px4_bridge.yaml")),
        DeclareLaunchArgument("with_map_recorder", default_value="true"),
        DeclareLaunchArgument("with_mission_gate", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("start_xrce_agent", default_value="false",
                              description="Start MicroXRCEAgent on the TELEM2 serial link"),
        DeclareLaunchArgument("xrce_dev", default_value="/dev/ttyTHS1",
                              description="Jetson UART wired to Pixhawk 6C TELEM2 (check your carrier board)"),
        DeclareLaunchArgument("xrce_baud", default_value="921600"),
    ]

    actions = [
        include("static_tf.launch.py", {"tf_config": L("tf_config")}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "livox_mid360.launch.py")),
            launch_arguments={"livox_config": L("livox_config")}.items(),
            condition=IfCondition(L("with_livox")),
        ),
        include("fast_lio.launch.py", {"fast_lio_config": L("fast_lio_config"), "map_dir": L("map_dir"),
                                       "use_sim_time": L("use_sim_time")}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("hummingbird_px4_bridge"), "launch", "px4_bridge.launch.py")),
            launch_arguments={"bridge_config": L("bridge_config")}.items(),
            condition=IfCondition(L("with_px4_bridge")),
        ),
        Node(
            package="hummingbird_bringup", executable="map_recorder", name="hummingbird_map_recorder",
            parameters=[{"output_dir": L("map_dir"), "voxel_size": L("map_voxel"),
                         "use_sim_time": L("use_sim_time")}],
            condition=IfCondition(L("with_map_recorder")), output="screen",
        ),
        Node(
            package="hummingbird_task_agent", executable="mission_gate", name="hummingbird_mission_gate",
            condition=IfCondition(L("with_mission_gate")), output="screen",
        ),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", L("xrce_dev"), "-b", L("xrce_baud")],
            condition=IfCondition(L("start_xrce_agent")), output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", os.path.join(share, "rviz", "mapping.rviz")],
            condition=IfCondition(L("rviz")),
        ),
    ]
    return LaunchDescription(args + actions)
