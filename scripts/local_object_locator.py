#!/usr/bin/python
from functools import partial
from operator import is_not
from sys import excepthook

import numpy as np
import rospy
from rospy import Header, Publisher, Subscriber
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo
from sensor_msgs import point_cloud2
import tf
import traceback
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
    

    def __init__(self, pub_name="/object_detector/objects_pointcloud", sub_name="/object_detector/object_detected"):
        rospy.init_node("local_object_locator", log_level=rospy.INFO)
        
        self.pc_pub = Publisher(pub_name, data_class=PointCloud2,queue_size=10)
        self.img_sub = Subscriber(sub_name, data_class=ObjectsInImg, callback=self.detect_objects_in_img)

        self.camera_model = image_geometry.PinholeCameraModel()
        self.bridge = CvBridge()

        self.tf_listener = tf.TransformListener()
    
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
        header.stamp = objects_in_im.im_color.header.stamp

        point_cloud = point_cloud2.create_cloud(header, fields, world_positions)

        self.pc_pub.publish(point_cloud)

    
    def get_pixel_positions_from_objects_detected(self, objects_detected):
        return [object_detected.bbox.center for object_detected in objects_detected]



    def get_cam_coords_from_img_positions(self, im_pixel_positions, im_depth, im_color, camera_info):
        coords = np.array([], dtype=float)
        coords.shape=(-1,3)


        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(camera_info)


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

        image_coords = []
        depth_coords = []

        for pixel_position in im_pixel_positions:
        
            image_coord = (pixel_position.y, pixel_position.x)
            image_coords.append(image_coord)
            # "map" from color to depth image
            depth_coord = (image_depth.shape[0]/2 + (image_coord[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
                            image_depth.shape[1]/2 + (image_coord[1] - image_color.shape[1]/2)*self.color2depth_aspect)
            depth_coords.append(depth_coord)
     


            self.tf_listener.waitForTransform('map', camera_info.header.frame_id, im_color.header.stamp,rospy.Duration.from_sec(1))

            for depth_coord, image_coord in zip(depth_coords, image_coords):
                try:
                    # get the depth reading at the centroid location
                    depth_value = image_depth[int(depth_coord[0]), int(depth_coord[1])] # you might need to do some boundary checking first!

                    if depth_value > 100: continue

                    # calculate object's 3d location in camera coords
                    camera_coord = camera_model.projectPixelTo3dRay((image_coord[1], image_coord[0])) #project the image coords (x,y) into 3D ray in camera coords 
                    camera_coord = [x/camera_coord[2] for x in camera_coord] # adjust the resulting vector so that z = 1
                    camera_coord = [x*depth_value for x in camera_coord] # multiply the vector by depth

                    #define a point in camera coordinates
                    object_location = PoseStamped()
                    object_location.header.stamp = im_color.header.stamp
                    object_location.header.frame_id = camera_info.header.frame_id
                    rospy.logdebug_throttle(5, "frame_id: %s"%(camera_info.header.frame_id))
                    object_location.pose.orientation.w = 1.0
                    object_location.pose.position.x = camera_coord[0]
                    object_location.pose.position.y = camera_coord[1]
                    object_location.pose.position.z = camera_coord[2]

                    object_location_map = self.tf_listener.transformPose('map', object_location)
                    obj_pos = object_location_map.pose.position

                    coords = np.append(coords, [[obj_pos.x, obj_pos.y, obj_pos.z]], axis=0)
                    rospy.logdebug_throttle(5, "world coordinates: " + str([[obj_pos.x, obj_pos.y, obj_pos.z]]))
                except IndexError as e:
                    continue
                except Exception as e:
                    rospy.logerr_throttle(5, traceback.print_exc())
                    rospy.logerr_throttle(5,"Transformation to deph or camera corrdinates failed!")
                    rospy.logerr_throttle(5,"\t \t Color, \t \t image")
                    rospy.logerr_throttle(5,"size:\t" + str(image_color.shape) + "," + str(image_depth.shape))
                    rospy.logerr_throttle(5,"coord:\t" + "(" + str(image_coord[0]) + "," + str(image_coord[1]) + "),\t" + str(depth_coord))
                    continue

        if self.visualisation and im_color is not None:
                    # covert images to open_cv
            try:
                # draw circles
                for image_coord, depth_coord in zip(image_coords, depth_coords):
                    cv2.circle(image_color, (int(image_coord[1]), int(image_coord[0])), 10, 255, -1)
                    cv2.circle(image_depth, (int(depth_coord[1]), int(depth_coord[0])), 5, 255, -1)

                #resize and adjust for visualisation
                image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
                image_depth *= 1.0/10.0 # scale for visualisation (max range 10.0 m)

                cv2.imshow("image depth", image_depth)
                cv2.imshow("image color", image_color)
                cv2.waitKey(1)
            except Exception as e:
                #rospy.logerr(e)
                rospy.logerr_throttle(5,e)

        return coords


    def get_cam_coord_from_img_position(self, im_pixel_position, im_depth, im_color, camera_info):
        """
        Attribution: the code is based on LCAS's  work (https://github.com/LCAS/CMP9767M/search?q=image_projection_3)
        """

        

if __name__ == "__main__":
    try:
        LocalObjectLocator()
        rospy.spin()

    except:
        pass

