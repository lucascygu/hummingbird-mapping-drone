#!/usr/bin/env bash
# Generated / assisted by Grok
# Pure-python unit tests (no ROS runtime needed; numpy + pyyaml + pytest required).
# The in-process ROS node tests in the same folders skip themselves here; run those with
#   colcon test --packages-select hummingbird_task_agent hummingbird_px4_bridge
# and the hardware-free end-to-end check with scripts/sim_e2e_test.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1   # ROS' launch_testing plugin breaks with newer pytest
for pkg in hummingbird_px4_bridge hummingbird_bringup hummingbird_task_agent; do
  (cd "$ROOT/ros2_ws/src/$pkg" && python3 -m pytest -q test)
done
