std_msgs/Header header

# Infromation on detected objects
ros_crop_estimation/ObjectDetected[] objects_detected


# Context information for detected objects
# Might be empy if no objects are detected.
sensor_msgs/CameraInfo camera_info_color
sensor_msgs/Image im_color
sensor_msgs/Image im_depth