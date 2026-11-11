# Generated / assisted by Grok
# Extended by Claude: max_speed_mps, approval note
# A high-level task issued by a ground station or teammate.
# This is not an actuator command.
#
# A GroundTask carries NO arm / takeoff approval on purpose: a ground robot or planner cannot
# approve its own task. mission_gate parks a valid task as a draft and reports
# "pending_approval" on /hummingbird/status; an operator releases it with the
# /hummingbird/mission/approve service (hummingbird_interfaces/ApproveMission).
# rtl and land are never parked: they are released immediately (failsafe priority).

std_msgs/Header header

string task_id
string task_type    # explore | map_region | inspect | loiter | rtl | land
string description
string frame_id     # e.g. hummingbird_map ; must be filled (not needed for rtl / land)

# Waypoints in frame_id (ENU if using the ROS convention).
# Identity (or all-zero) orientation = heading unspecified.
geometry_msgs/Pose[] waypoints

# 0 = use the site geofence configured in mission_gate. A positive value can only tighten it.
float64 geofence_max_altitude_m
float64 geofence_radius_m

# Optional speed cap in m/s. 0 = unspecified. Must not exceed the gate's site speed limit.
float64 max_speed_mps
