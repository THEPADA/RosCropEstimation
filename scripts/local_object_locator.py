#!/usr/bin/python
from functools import partial
from operator import is_not

import numpy as np
import rospy
from rospy import Header, Publisher, Subscriber
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from sensor_msgs import point_cloud2
import tf
import image_geometry
from ros_crop_estimation.msg import ObjectDetected, ObjectsInImg
from cv_bridge import CvBridge, CvBridgeError
import cv2

class LocalObjectLocator:
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
    camera_model = None
    tf_listener = None
    

    def __init__(self, model_repo_or_dir='ultralytics/yolov5', model_name = 'yolov5s', pub_name="/object_detector/objects_pointcloud", sub_name="/object_detector/object_detected"):
        rospy.init_node("ObjectLocator", log_level=rospy.INFO)
        
        self.pc_pub = Publisher(pub_name, data_class=PointCloud2,queue_size=10)
        self.img_sub = Subscriber(sub_name, data_class=ObjectsInImg, callback=self.detect_objects_in_img)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.bridge = CvBridge()

        self.tf_listener = tf.TransformListener()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_front_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once
    
    def detect_objects_in_img(self, objects_in_im):

        rospy.logdebug("detected objects!")

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

        point_cloud = point_cloud2.create_cloud(header, fields, world_positions)

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
            rospy.logerr("Transformation to deph or camera corrdinates failed!")
            rospy.logerr("\t \t Color, \t \t image")
            rospy.logerr("size:\t" + str(image_color.shape) + "," + str(image_depth.shape))
            rospy.logerr("coord:\t" + "(" + str(im_pixel_position.x) + "," + str(im_pixel_position.y) + "),\t" + str(depth_coords))
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
        LocalObjectLocator()
        rospy.spin()

    except:
        pass

