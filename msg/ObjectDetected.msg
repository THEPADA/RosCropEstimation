std_msgs/Header header

# The unique numeric ID of the object class. To get additional information about
# this ID, such as its human-readable class name, listeners should perform a
# lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
int64 id

# The probability or confidence value of the detected object. By convention,
# this value should lie in the range [0-1].
float64 score

# View 1: BoundingBox
vision_msgs/BoundingBox2D bbox

# View 2: This represents a pose in free space with uncertainty.
geometry_msgs/Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance