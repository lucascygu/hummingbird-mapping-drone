#!/usr/bin/env bash
# Generated / assisted by Claude
# End-to-end test with no hardware: a fake Mid-360 flies a known lap through a known room, and the
# real stack (FAST-LIO2 -> PX4 bridge + map_recorder + map_tiler) is scored against ground truth.
#
# Needs a built and sourced workspace that includes fast_lio and livox_ros_driver2:
#   source ros2_ws/install/setup.bash
#   bash scripts/sim_e2e_test.sh                        # ~1 min, exit code 0 = pass
#   bash scripts/sim_e2e_test.sh 46 lidar_clock:=boot   # unsynced LiDAR clock (bridge uses receipt time)
#
# There is no PX4 in this test. It checks the estimator INPUT the bridge would send, not the flight.
set -euo pipefail
DURATION="${1:-46}"
shift || true
OUT="$(mktemp -d -t hummingbird_sim_XXXXXX)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"   # keep clear of a real vehicle on the default domain

# `timeout` is not only a safety net. A background job of a non-interactive shell inherits SIGINT
# as "ignored", and ros2 launch then cannot be stopped cleanly; timeout hands its child a default
# SIGINT disposition and forwards the signal to the whole process group.
timeout --signal=INT --kill-after=45 "$((DURATION + 90))" \
  ros2 launch hummingbird_bringup sim_mapping.launch.py map_dir:="$OUT/maps" "$@" > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  echo "logs and maps: $OUT"
}
trap cleanup EXIT

sleep 3
ros2 run hummingbird_bringup sim_check --ros-args -p duration_s:="$DURATION"
