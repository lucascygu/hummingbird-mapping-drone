# Generated / assisted by Grok
# Extended by Claude: action, mission_type, sequence / total
# The only motion request the task layer may publish after gating.
# Consumers (navigation / PX4 bridge) must still apply their own limits.
# Do not treat this as an arm, takeoff, or actuator command.

std_msgs/Header header

string mission_id
string mission_type   # explore | map_region | inspect | loiter | rtl | land
string frame_id       # frame of pose; may be empty for ACTION_RTL / ACTION_LAND

# What the consumer is asked to do with this setpoint.
uint8 ACTION_GOTO = 0     # fly to pose
uint8 ACTION_LOITER = 1   # fly to pose and hold there
uint8 ACTION_RTL = 2      # return to launch; pose is ignored
uint8 ACTION_LAND = 3     # land at the current position; pose is ignored
uint8 action

# A mission is published as `total` setpoints with sequence 0 .. total-1, in order.
# A consumer must not start a mission until it holds all of them, and a setpoint with a new
# mission_id supersedes every setpoint of the previous mission.
uint32 sequence
uint32 total

geometry_msgs/Pose pose

# Optional speed cap in m/s. 0 means "unspecified, use the tracker default".
float64 max_speed_mps
# True if pose.orientation carries a requested heading (yaw about +z of frame_id).
# False: keep the current heading; pose.orientation is then the identity.
bool yaw_valid
