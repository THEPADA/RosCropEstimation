#!/usr/bin/python3
import torch
import rospy
import os
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose2D
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


from ws02_robot_control.msg import ObjectDetected, ObjectsInImg
from vision_msgs.msg import BoundingBox2D


import logging
import sys
import PIL.Image
import message_filters
import pyrealsense2


logging.basicConfig(filename='YoloObjectLocator.log', level=logging.DEBUG, filemode='w', format='%(name)s - %(levelname)s - %(message)s')

class ObjectDetector:
    """
    The YoloObjectLocator publishes the detected poses of grape-bunches in the image frame.
    This is done by subscribing to the robot's camera and extracting grape-bunch image positions from the image.
    """

    det_pub = None
    img_sub = None
    detected_positions = None
    detector_model = None
    cv_bridge = None

    def __init__(self, model_repo_or_dir='ultralytics/yolov5', model_name = 'yolov5s', pub_name="/object_detector/object_detected"):
        rospy.init_node("Dectector")
        
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../models/yolov5/best.pt')

        try:
            self.detector_model = torch.hub.load('ultralytics/yolov5', 'custom', filename)
        except:
            logging.exception("Failed to load model from torch hub")
            t, e = sys.exc_info()[:2]
            print("ERROR: could not load model for object detection")
            print(e)
            exit()

        self.det_pub = rospy.Publisher(pub_name, data_class=ObjectsInImg, queue_size=10)
        self.color_img_sub = message_filters.Subscriber("thorvald_001/kinect2_front_camera/hd/image_color_rect", Image)
        self.depth_img_sub = message_filters.Subscriber("thorvald_001/kinect2_front_sensor/sd/image_depth_rect", Image)
        self.camera_info = message_filters.Subscriber("thorvald_001/kinect2_front_sensor/sd/camera_info", CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([self.color_img_sub, self.depth_img_sub, self.camera_info], 10, 0.01, allow_headerless=True)
        ts.registerCallback(self.detect_objects_in_img)

    def detect_objects_in_img(self, image_message, depth_message, camera_info_message):
        timestamp = image_message.header.stamp
        im = np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, -1)

        #im2 = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        #cv2.imshow("Recorded image",im2)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()

        objects = self.get_objects_in_img_from_detector(im)
        df = None
        if objects is not None:
            df = objects.pandas().xyxy[0]
            # If objects detected publish results as poses
        
        detections = self.serialise_detections_to_object_detected_msg(df)
        object_in_img = self.serialise_detections_msgs_to_object_in_img_msg(image_message, depth_message, camera_info_message, detections)

        self.det_pub.publish(object_in_img)



    def serialise_detections_msgs_to_object_in_img_msg(self, image_message, depth_message, camera_info_message, detections):
        # std_msgs/Header header

        # # Infromation on detected objects
        # ws02_robot_control/ObjectDetected[] objects_detected


        # # Context information for detected objects
        # # Might be empy if no objects are detected.
        # sensor_msgs/CameraInfo camera_info_color
        # sensor_msgs/Image im_color
        # sensor_msgs/Image im_depth
        if image_message is None or depth_message is None or camera_info_message is None or detections is None:
            return ObjectsInImg()
        
        im_detection_msg = ObjectsInImg()
        im_detection_msg.objects_detected = detections
        im_detection_msg.camera_info_color = camera_info_message
        im_detection_msg.im_color = image_message
        im_detection_msg.im_depth = depth_message
        return im_detection_msg

    def serialise_detections_to_object_detected_msg(self, df):
        return [self.serialise_detection_to_object_detected_msg(detection) for index, detection in df.iterrows()]

    def serialise_detection_to_object_detected_msg(self, df):
        # # Rows: xmin, ymin, xmax, ymax,  confidence, class, name, xcenter, ycenter, xsize, ysize
        # std_msgs/Header header

        # # The unique numeric ID of the object class. To get additional information about
        # # this ID, such as its human-readable class name, listeners should perform a
        # # lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
        # int64 id

        # # The probability or confidence value of the detected object. By convention,
        # # this value should lie in the range [0-1].
        # float64 score

        # # View 1: BoundingBox
        # vision_msgs/BoundingBox2D bbox

        # # View 2: This represents a pose in free space with uncertainty.
        # geometry_msgs/Pose pose

        # # Row-major representation of the 6x6 covariance matrix
        # # The orientation parameters use a fixed-axis representation.
        # # In order, the parameters are:
        # # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        # float64[36] covariance

        if df.empty or df is None:   
            return ObjectDetected()

        #df['xmin'],df['xmax']=np.where(df['xmin']>df['xmax'],(df['xmax'],df['xmin']),(df['xmin'],df['xmax']))
        #df['ymin'],df['ymax']=np.where(df['ymin']>df['ymax'],(df['ymax'],df['ymin']),(df['ymin'],df['ymax']))

        df["xcenter"] = (df["xmax"]+ df["xmin"])/2
        df["ycenter"] = (df["ymax"]+ df["ymin"])/2

        df["xsize"] = df["xmax"] - df["xmin"]
        df["ysize"] = df["ymax"] - df["ymin"]


        object_detected_msg = ObjectDetected()
        object_detected_msg.id = df["class"]
        object_detected_msg.score = df["confidence"]
        object_detected_msg.bbox = BoundingBox2D(Pose2D(df["xcenter"], df["ycenter"], 0.), df["xsize"], df["ysize"])

        return object_detected_msg
            

    def get_objects_in_img_from_detector(self, img):
        if self.detector_model is not None:
            object_detected = self.detector_model(img)
            return object_detected
        rospy.logdebug("Object detector model is not ready yet. WAITING...")
        return []
        
if __name__ == "__main__":
    try:
        ObjectDetector()
        rospy.spin()

    except Exception as e:
        print(e)
        pass

