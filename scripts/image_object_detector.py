#!/usr/bin/python3
import torch
import rospy
import os
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, CameraInfo
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
        im = np.frombuffer(image_message.data, dtype=np.uint8).reshape(image_message.height, image_message.width, -1)
        objects = self.get_objects_in_img_from_detector(im)
        df = None
      
        if objects is not None:
            df = objects.pandas().xyxy[0]
        
        detections = self.serialise_detections_to_object_detected_msg(df)
        object_in_img = self.serialise_detections_msgs_to_object_in_img_msg(image_message, depth_message, camera_info_message, detections)
        self.det_pub.publish(object_in_img)

    def serialise_detections_msgs_to_object_in_img_msg(self, image_message, depth_message, camera_info_message, detections):
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
        if df.empty or df is None:   
            return ObjectDetected()

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

