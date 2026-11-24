# Published by behavior_node, consumed by vehicle_interface_node.

uint8 STATE_UNKNOWN   = 255
uint8 STATE_SEARCH    = 0
uint8 STATE_TRACK     = 1
uint8 STATE_REACQUIRE = 2
uint8 STATE_LAND      = 3

std_msgs/Header header

uint8 state

# The detection that justified this state. Only meaningful when
# target_valid is true. Bundled with state so the two cannot disagree.
bool target_valid
float32 center_x     # pixels
float32 center_y     # pixels
float32 size_x       # pixels
float32 size_y       # pixels
float32 image_width  # pixels
float32 image_height # pixels
float32 score
