#!/usr/bin/env bash
# Pure-python unit tests (no ROS runtime needed; numpy + pyyaml + pytest required).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1   # ROS' launch_testing plugin breaks with newer pytest
for pkg in hummingbird_px4_bridge hummingbird_bringup hummingbird_task_agent; do
  (cd "$ROOT/ros2_ws/src/$pkg" && python3 -m pytest -q test)
done
