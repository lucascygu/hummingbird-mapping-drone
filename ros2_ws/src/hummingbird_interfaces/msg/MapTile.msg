# A compact map fragment shared from air to ground.
# data interpretation depends on data_kind; keep tiles small.

std_msgs/Header header

string tile_id
string frame_id
string data_kind    # occupancy | height | other

float64 resolution
uint32 width
uint32 height
float64 origin_x
float64 origin_y
float64 origin_z

int16[] data
