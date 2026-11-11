# The only motion request the task layer may publish after gating.
# Consumers (navigation / PX4 bridge) must still apply their own limits.
# Do not treat this as an arm, takeoff, or actuator command.

std_msgs/Header header

string mission_id
string frame_id
geometry_msgs/Pose pose

# Optional speed cap in m/s. 0 means "unspecified, use the tracker default".
float64 max_speed_mps
bool yaw_valid
