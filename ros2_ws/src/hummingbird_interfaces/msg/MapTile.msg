# Generated / assisted by Grok
# Extended by Claude: layout of the "height" data_kind (published by hummingbird_bringup/map_tiler)
# A compact map fragment shared from air to ground.
# data interpretation depends on data_kind; keep tiles small.
#
# data_kind "height": 2.5D height map. data[row * width + col] is the highest point seen in the
# cell covering x in [origin_x + col*resolution, origin_x + (col+1)*resolution) and
# y in [origin_y + row*resolution, origin_y + (row+1)*resolution), as centimetres above origin_z.
# -32768 = nothing observed in that cell. A tile with the same tile_id replaces the earlier one.

std_msgs/Header header

string tile_id      # stable per tile, e.g. "height/<ix>/<iy>"
string frame_id     # frame of origin_x/y/z (the FAST-LIO world frame, camera_init, by default)
string data_kind    # occupancy | height | other

float64 resolution  # metres per cell
uint32 width        # cells along +x
uint32 height       # cells along +y
float64 origin_x
float64 origin_y
float64 origin_z

int16[] data
