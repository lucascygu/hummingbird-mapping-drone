# Generated / assisted by Grok
"""Config sanity checks (no ROS needed)."""

import ipaddress
import json
import pathlib

import yaml

CFG = pathlib.Path(__file__).resolve().parents[1] / "config"


def test_mid360_json_is_valid_and_same_subnet():
    c = json.loads((CFG / "MID360_config.json").read_text())
    assert c["lidar_summary_info"]["lidar_type"] == 8
    host = ipaddress.ip_address(c["MID360"]["host_net_info"]["point_data_ip"])
    lidar = ipaddress.ip_address(c["lidar_configs"][0]["ip"])
    net = ipaddress.ip_network("192.168.1.0/24")
    assert host in net and lidar in net and host != lidar
    assert c["lidar_configs"][0]["pcl_data_type"] == 1


def test_fast_lio_yaml_matches_livox_driver():
    p = yaml.safe_load((CFG / "fast_lio_mid360.yaml").read_text())["/**"]["ros__parameters"]
    assert p["common"]["lid_topic"] == "/livox/lidar"
    assert p["common"]["imu_topic"] == "/livox/imu"
    assert p["preprocess"]["lidar_type"] == 1
    assert p["mapping"]["extrinsic_T"] == [-0.011, -0.02329, 0.04412]
    assert len(p["mapping"]["extrinsic_R"]) == 9
    assert p["pcd_save"]["pcd_save_en"] is True
    # map_recorder builds the saved map from /cloud_registered: it must carry the dense scan.
    assert p["publish"]["scan_publish_en"] is True and p["publish"]["dense_publish_en"] is True


def test_static_tf_lidar_offset_matches_fast_lio_extrinsic():
    tf = yaml.safe_load((CFG / "static_tf.yaml").read_text())
    p = yaml.safe_load((CFG / "fast_lio_mid360.yaml").read_text())["/**"]["ros__parameters"]
    assert tf["body_to_livox_frame"]["xyz"] == p["mapping"]["extrinsic_T"]


def test_bridge_and_static_tf_agree_on_imu_to_base():
    bridge = pathlib.Path(__file__).resolve().parents[2] / "hummingbird_px4_bridge" / "config" / "px4_bridge.yaml"
    b = yaml.safe_load(bridge.read_text())["hummingbird_px4_odom_bridge"]["ros__parameters"]
    tf = yaml.safe_load((CFG / "static_tf.yaml").read_text())
    assert b["imu_to_base_xyz"] == tf["body_to_base_link"]["xyz"]
    assert b["imu_to_base_rpy"] == tf["body_to_base_link"]["rpy"]
    assert b["output_topic"] == "/fmu/in/vehicle_visual_odometry"
