#!/usr/bin/env bash
# Save the current map. Run in a sourced ROS 2 shell while mapping.launch.py is running.
#   scripts/save_map.sh            -> map_recorder voxel map (always available)
#   scripts/save_map.sh fastlio    -> FAST-LIO /map_save (needs publish.map_en: true)
#   scripts/save_map.sh both
set -euo pipefail
mode="${1:-recorder}"
if [[ "$mode" == recorder || "$mode" == both ]]; then
  ros2 service call /hummingbird/save_map std_srvs/srv/Trigger "{}"
fi
if [[ "$mode" == fastlio || "$mode" == both ]]; then
  ros2 service call /map_save std_srvs/srv/Trigger "{}"
fi
echo "Downsample/export example:"
echo "  ros2 run hummingbird_bringup pcd_export ~/hummingbird_maps/<file>.pcd ~/hummingbird_maps/map_10cm.pcd --voxel 0.10"
