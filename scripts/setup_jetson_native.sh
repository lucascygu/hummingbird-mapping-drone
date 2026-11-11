#!/usr/bin/env bash
# Generated / assisted by Grok
# Reviewed by Claude: rosdep initialisation; NOT run on a Jetson by either assistant.
# Native (non-docker) setup for a Jetson running Ubuntu 22.04 + ROS 2 Humble.
# The team's report records a Jetson Orin Nano Super 8GB on JetPack 6.2 (Ubuntu 22.04, aarch64)
# with ROS 2 Humble. ASSUMED: Humble installed from packages.ros.org, user in the dialout group
# for the TELEM2 UART.
#
#   bash scripts/setup_jetson_native.sh            # installs deps, Livox-SDK2, XRCE agent, builds ws
#   SKIP_SYSTEM=1 bash scripts/setup_jetson_native.sh   # only (re)build the workspace
set -euo pipefail
REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS="${REPO_DIR}/ros2_ws"
LIVOX_SDK2_REF=c0796f04c143143899c87a773d9f6b7136453c0b
XRCE_AGENT_REF=v2.4.2

if [[ "${SKIP_SYSTEM:-0}" != 1 ]]; then
  sudo apt-get update
  sudo apt-get install -y git build-essential cmake python3-vcstool python3-colcon-common-extensions \
    python3-numpy python3-yaml python3-pytest python3-rosdep libpcl-dev libeigen3-dev \
    ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-tf2-ros ros-humble-sensor-msgs-py \
    ros-humble-std-srvs ros-humble-nav-msgs

  tmp="$(mktemp -d)"
  git clone https://github.com/lucascygu/Livox-SDK2.git "$tmp/Livox-SDK2"
  (cd "$tmp/Livox-SDK2" && git checkout "$LIVOX_SDK2_REF" && mkdir build && cd build && cmake .. && make -j"$(nproc)" && sudo make install)
  git clone -b "$XRCE_AGENT_REF" https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$tmp/xrce"
  (cd "$tmp/xrce" && mkdir build && cd build && cmake .. && make -j"$(nproc)" && sudo make install)
  sudo ldconfig
fi

cd "$WS"
mkdir -p src
vcs import --recursive src < hummingbird.repos
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
rm -rf src/livox_ros_driver2/launch && cp -r src/livox_ros_driver2/launch_ROS2 src/livox_ros_driver2/launch

set +u; source /opt/ros/humble/setup.bash; set -u
# rosdep needs its source list once per machine; without it the install below fails silently.
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble --skip-keys "livox_ros_driver2 fast_lio" \
  || echo "WARNING: rosdep could not install every dependency; the build below may fail" >&2
# Orin NX/Nano: 2 workers avoids OOM while compiling FAST-LIO / px4_msgs.
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

cat <<MSG
Done. Next:
  source ${WS}/install/setup.bash
  bash ${REPO_DIR}/scripts/sim_e2e_test.sh     # no hardware needed: checks the whole mapping chain
  # Jetson NIC for the Mid-360 must be 192.168.1.50/24 (or edit config/MID360_config.json)
  ros2 launch hummingbird_bringup mapping.launch.py start_xrce_agent:=true xrce_dev:=/dev/ttyTHS1
MSG
