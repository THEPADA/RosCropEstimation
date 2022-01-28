#!/usr/bin/python3
import traceback
from attr import dataclass
import torch
import rospy
import os
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image, CameraInfo
import numpy as np


from ros_crop_estimation.msg import ObjectDetected, ObjectsInImg
from vision_msgs.msg import BoundingBox2D


import logging
import sys
import PIL.Image
import message_filters
import pyrealsense2


logging.basicConfig(filename='YoloObjectLocator.log', level=logging.DEBUG,
                    filemode='w', format='%(name)s - %(levelname)s - %(message)s')


class ObjectDetector:
    """
    The ObjectDetector publishes the detected poses of grape-bunches in the image frame.
    This is done by subscribing to the robot's camera and extracting grape-bunch image positions from the image.
    This implementation features yolov5!
    """

    det_pub = None
    img_sub = None
    detected_positions = None
    detector_model = None
    cv_bridge = None

    def __init__(self,
                 model_repo_or_dir='ultralytics/yolov5',
                 model_name=['custom', '../models/yolov5/best.pt'],
                 pub_name="/object_detector/object_detected"):
        rospy.init_node("image_object_detector", log_level=rospy.DEBUG)

        rospy.loginfo("model_repo_or_dir: " + str(model_repo_or_dir))
        model_repo_or_dir = rospy.get_param("~model_repo_or_dir", model_name)
        rospy.loginfo("model_repo_or_dir: " + str(model_repo_or_dir))
        model_name = rospy.get_param("~model_name", model_name)
        img_color_top = rospy.get_param("~color_img_topic")
        img_depth_top = rospy.get_param("~depth_img_topic")
        camera_info_top = rospy.get_param("~camera_info_topic")

        try:
            self.detector_model = torch.hub.load(model_repo_or_dir, *model_name)
        except:
            rospy.logerr("Failed to load model from torch hub")
            rospy.logerr(traceback.format_exc())
            exit()
        
        self.det_pub = rospy.Publisher(
            pub_name, data_class=ObjectsInImg, queue_size=1)
        self.color_img_sub = message_filters.Subscriber(img_color_top, data_class=Image)
        self.depth_img_sub = message_filters.Subscriber(img_depth_top, data_class=Image)
        self.camera_info = message_filters.Subscriber(camera_info_top, data_class=CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_img_sub, self.depth_img_sub, self.camera_info], 10, 0.01)
        ts.registerCallback(self.detect_objects_in_img)

    def detect_objects_in_img(self, image_message, depth_message, camera_info_message):
        """
        This function is called when a new image is received.
        It extracts the grape-bunch image positions from the image and publishes them.
        """

        im = np.frombuffer(image_message.data, dtype=np.uint8).reshape(
            image_message.height, image_message.width, -1)
        objects = self.get_objects_in_img_from_detector(im)
        df = None

        if objects is not None:
            df = objects.pandas().xyxy[0]

        detections = self.serialise_detections_to_object_detected_msg(df)
        object_in_img = self.serialise_detections_msgs_to_object_in_img_msg(
            image_message, depth_message, camera_info_message, detections)
        self.det_pub.publish(object_in_img)

    def serialise_detections_msgs_to_object_in_img_msg(self, image_message, depth_message, camera_info_message, detections):
        """
        Serialises the detections to an object_in_img message
        @param detections: list of ObjectDetected messages
        @return: ObjectsInImg message
        @rtype: ObjectsInImg
        """
        if image_message is None or depth_message is None or camera_info_message is None or detections is None:
            return ObjectsInImg()

        im_detection_msg = ObjectsInImg()
        im_detection_msg.objects_detected = detections
        im_detection_msg.camera_info_color = camera_info_message
        im_detection_msg.im_color = image_message
        im_detection_msg.im_depth = depth_message
        return im_detection_msg

    def serialise_detections_to_object_detected_msg(self, df):
        """
        Serialises the detections to object_detected message.
        @param df: pandas dataframe with detections
        @return: ObjectDetected message
        """
        return [self.serialise_detection_to_object_detected_msg(detection) for index, detection in df.iterrows()]

    def serialise_detection_to_object_detected_msg(self, df):
        """
        Serialises the detection to an object_detected message.
        @param df: pandas dataframe with detection
        @return: ObjectDetected message
        """

        if df.empty or df is None:
            return ObjectDetected()

        df["xcenter"] = (df["xmax"] + df["xmin"])/2
        df["ycenter"] = (df["ymax"] + df["ymin"])/2

        df["xsize"] = df["xmax"] - df["xmin"]
        df["ysize"] = df["ymax"] - df["ymin"]

        object_detected_msg = ObjectDetected()
        object_detected_msg.id = df["class"]
        object_detected_msg.score = df["confidence"]
        object_detected_msg.bbox = BoundingBox2D(
            Pose2D(df["xcenter"], df["ycenter"], 0.), df["xsize"], df["ysize"])

        return object_detected_msg

    def get_objects_in_img_from_detector(self, img):
        """
        Extracts the grape-bunch image positions from the image.
        @param img: image
        @return: pandas dataframe with detections
        """
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
        rospy.logerr(traceback.format_exc())
        pass
