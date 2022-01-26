#!/usr/bin/python
from functools import partial
from operator import is_not
from sqlite3 import Timestamp
import struct
from sys import float_repr_style
from unittest import skip

import numpy as np
from numpy import dtype
import rospy
from rospy import Header, Publisher, Subscriber
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import PointCloud2, Image, PointField, CameraInfo
from sensor_msgs import point_cloud2
import logging
import tf
import tf2_geometry_msgs
import image_geometry
from ws02_robot_control.msg import ObjectDetected, ObjectsInImg
from cv_bridge import CvBridge, CvBridgeError
import cv2


logging.basicConfig(filename='YoloObjectLocator.log', level=logging.DEBUG, filemode='w', format='%(name)s - %(levelname)s - %(message)s')






class YoloObjectLocator:
    """
    The YoloObjectLocator publishes a point cloud that includes the detected positions of grape-bunches.
    This is done by subscribing to the robot's camera and extracting grape image positions from the image.
    These image positions are then used to infer the position of the grape using image-to-world transforms.
    """

    pc_pub = None
    img_sub = None
    detected_positions = None
    detector_model = None
    color2depth_aspect = (84.1/1920) / (70.0/512)
    visualisation = True
    points = None
    camera_model = None
    tf_listener = None
    

    def __init__(self, model_repo_or_dir='ultralytics/yolov5', model_name = 'yolov5s', pub_name="/object_detector/objects_pointcloud", sub_name="/object_detector/object_detected"):
        rospy.init_node("ObjectLocator", log_level=rospy.INFO)
        
        self.pc_pub = Publisher(pub_name, data_class=PointCloud2,queue_size=10)
        self.img_sub = Subscriber(sub_name, data_class=ObjectsInImg, callback=self.detect_objects_in_img)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.bridge = CvBridge()

        self.tf_listener = tf.TransformListener()
        self.points = np.array([],dtype=float)
        self.points.shape = (-1,3)

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_front_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def add_new_detected_points(self, new_points):
        print("prior points" + str(len(self.points)))
        for new_point in new_points:
            if len(filter(lambda x: not(np.isnan(x)), new_point)) != 3: continue
            if self.get_close_point_indices(new_point, self.points) > 0.018:
                self.points = np.vstack((self.points, new_point))

        print("new points" + str(len(new_points)))
        print("posterior points" + str(len(self.points)))

    def get_close_point_indices(self, node, nodes):
        if len(nodes) == 0: return np.inf
        nodes = np.asfarray(nodes)
        dist_2 = np.sum((nodes - node)**2, axis=1)
        print(dist_2)
        return dist_2[np.argmin(dist_2)]
    
    def detect_objects_in_img(self, objects_in_im):

        rospy.logdebug("detected objects!")
        # std_msgs/Header header

        # # Infromation on detected objects
        # ws02_robot_control/ObjectDetected[] objects_detected


        # # Context information for detected objects
        # # Might be empy if no objects are detected.
        # sensor_msgs/CameraInfo camera_info_color
        # sensor_msgs/Image im_color
        # sensor_msgs/Image im_depth

        object_positions = self.get_pixel_positions_from_objects_detected(objects_in_im.objects_detected)

        world_positions = self.get_cam_coords_from_img_positions(object_positions, 
            objects_in_im.im_depth, objects_in_im.im_color, objects_in_im.camera_info_color)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          #PointField('rgba', 12, PointField.UINT32, 1),
          ]

        header = Header()
        header.frame_id = "map"

        self.add_new_detected_points(world_positions)
        point_cloud = point_cloud2.create_cloud(header, fields, self.points)

        self.pc_pub.publish(point_cloud)

    
    def get_pixel_positions_from_objects_detected(self, objects_detected):
        return [object_detected.bbox.center for object_detected in objects_detected]

    def get_cam_coords_from_img_positions(self, im_pixel_positions, im_depth, im_color, camera_model):
        coords = [self.get_cam_coord_from_img_position(pixel_position, im_depth, im_color, camera_model) for pixel_position in im_pixel_positions]
        coords = filter(partial(is_not, None), coords)
        coords = np.array(coords, dtype=float)
        coords.shape=(-1,3)
        rospy.logdebug("len of coords:" + str(len(coords)))

        if self.visualisation and im_color is not None:
                    # covert images to open_cv
            image_color = None
            try:
                image_color = self.bridge.imgmsg_to_cv2(im_color, "bgr8")
                image_depth = self.bridge.imgmsg_to_cv2(im_depth, "32FC1")

                                # draw circles
                for pixel_position in im_pixel_positions:
                    cv2.circle(image_color, (int(pixel_position.x), int(pixel_position.y)), 10, 255, -1)
                #resize and adjust for visualisation
                image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
                cv2.imshow("image color", image_color)
                cv2.waitKey(1)
                return coords
            except CvBridgeError as e:
                #rospy.logerr(e)
                return None
            

           


    def get_cam_coord_from_img_position(self, im_pixel_position, im_depth, im_color, camera_model):
        """

        Attribution: the code is based on LCAS's  work (https://github.com/LCAS/CMP9767M/search?q=image_projection_3)
        """

        image_color = None
        image_depth = None

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(im_color, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(im_depth, "32FC1")

        except CvBridgeError as e:
            rospy.logerr(e)
            return None

        # "map" from color to depth image
        rospy.logdebug("image shapes: " + str(image_color.shape) + str(image_depth.shape))
        rospy.logdebug("color image pixel: " + str( im_pixel_position.x) + "," + str(im_pixel_position.y))
        depth_coords = (image_depth.shape[0]/2 + (im_pixel_position.x- image_color.shape[0]/2)*self.color2depth_aspect, 
            image_depth.shape[1]/2 + (im_pixel_position.y - image_color.shape[1]/2)*self.color2depth_aspect)

        try:
            # get the depth reading at the centroid location
            depth_value = image_depth[int(depth_coords[1]), int(depth_coords[0])] # you might need to do some boundary checking first!

            # calculate object's 3d location in camera coords
            camera_coords = self.camera_model.projectPixelTo3dRay((im_pixel_position.x, im_pixel_position.y)) #project the image coords (x,y) into 3D ray in camera coords 
            camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
            camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth
        except Exception as e:
            #rospy.logerr("Transformation to deph or camera corrdinates failed!")
            #rospy.logerr("\t \t Color, \t \t image")
            #rospy.logerr("size:\t" + str(image_color.shape) + "," + str(image_depth.shape))
            #rospy.logerr("coord:\t" + "(" + str(im_pixel_position.x) + "," + str(im_pixel_position.y) + "),\t" + str(depth_coords))
            return None
        #define a point in camera coordinates
        object_location = PoseStamped()
        object_location.header.stamp = im_color.header.stamp
        object_location.header.frame_id = "thorvald_001/kinect2_front_rgb_optical_frame"
        object_location.pose.orientation.w = 1.0
        object_location.pose.position.x = camera_coords[0]
        object_location.pose.position.y = camera_coords[1]
        object_location.pose.position.z = camera_coords[2]

        object_location_map = self.tf_listener.transformPose('map', object_location)
        obj_pos = object_location_map.pose.position

        return [obj_pos.x, obj_pos.y, obj_pos.z]

if __name__ == "__main__":
    try:
        YoloObjectLocator()
        rospy.spin()

    except:
        pass

