# Generated / assisted by Grok
# A high-level task issued by a ground station or teammate.
# This is not an actuator command.

std_msgs/Header header

string task_id
string task_type    # explore | map_region | inspect | loiter | rtl | land
string description
string frame_id     # e.g. hummingbird_map ; must be filled

# Waypoints in frame_id (ENU if using the ROS convention).
geometry_msgs/Pose[] waypoints

float64 geofence_max_altitude_m
float64 geofence_radius_m
