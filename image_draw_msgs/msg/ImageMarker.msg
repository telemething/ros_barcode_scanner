uint8 CIRCLE=0
uint8 LINE_STRIP=1
uint8 LINE_LIST=2
uint8 POLYGON=3
uint8 POINTS=4
uint8 TEXT=5

uint8 ADD=0
uint8 REMOVE=1

std_msgs/Header header
# Namespace which is used with the id to form a unique id.
string ns
# Unique id within the namespace.
int32 id

# One of the above types, e.g. CIRCLE, LINE_STRIP, etc.
uint8 type
# Either ADD or REMOVE.
uint8 action 0
# Two-dimensional coordinate position, in pixel-coordinates.
geometry_msgs/Point position

# The scale of the object, e.g. the diameter for a CIRCLE.
float32 scale 1.0
# The outline color of the marker.
std_msgs/ColorRGBA outline_color
# Whether or not to fill in the shape with color.
bool filled false
# Fill color; in the range: [0.0-1.0]
std_msgs/ColorRGBA fill_color

# How long the object should last before being automatically deleted.
# 0 indicates forever.
builtin_interfaces/Duration lifetime

# Coordinates in 2D in pixel coords. Used for LINE_STRIP, LINE_LIST, POINTS, etc.
geometry_msgs/Point[] points

# The color for each line, point, etc. in the points field.
std_msgs/ColorRGBA[] outline_colors

# The thickness of every drawn object.
int32 thickness 1

# The text to draw if using the TEXT type.
string text