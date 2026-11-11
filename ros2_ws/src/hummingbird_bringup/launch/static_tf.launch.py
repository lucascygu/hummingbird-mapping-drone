"""Static TFs: map -> camera_init (FAST-LIO world), body -> base_link, body -> livox_frame.

FAST-LIO itself broadcasts camera_init -> body (the Mid-360 IMU) at the LiDAR rate.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_PAIRS = {
    "map_to_camera_init": ("map", "camera_init"),
    "body_to_base_link": ("body", "base_link"),
    "body_to_livox_frame": ("body", "livox_frame"),
}


def _nodes(context):
    with open(LaunchConfiguration("tf_config").perform(context)) as fh:
        cfg = yaml.safe_load(fh)
    out = []
    for key, (parent, child) in _PAIRS.items():
        t = cfg[key]
        x, y, z = (str(v) for v in t["xyz"])
        r, p, yw = (str(v) for v in t["rpy"])
        out.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"tf_{parent}_to_{child}",
            arguments=["--x", x, "--y", y, "--z", z, "--roll", r, "--pitch", p, "--yaw", yw,
                       "--frame-id", parent, "--child-frame-id", child],
        ))
    return out


def generate_launch_description():
    share = get_package_share_directory("hummingbird_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("tf_config", default_value=os.path.join(share, "config", "static_tf.yaml")),
        OpaqueFunction(function=_nodes),
    ])
