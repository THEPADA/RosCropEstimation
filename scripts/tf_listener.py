from ctypes import pointer
import rospy
import math
import tf
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point
from std_msgs.msg import Header
import numpy as np

from tf import listener
from tf import transformations

class TFListener:
    def __init__(self):
        rospy.init_node("fancy_listener")
        #self.tf_sub = rospy.Subscriber("thorvald_001/kinect2_left_rgb_optical_frame", callback=self.show_cordinates, data_class=PoseStamped)
        self.listener = tf.TransformListener()
        self.pose_tf = "thorvald_001/kinect2_left_rgb_optical_frame"
        self.pose_pub = rospy.Publisher("test_pose",PoseStamped, queue_size=1)
        self.rate = rospy.Rate(.5)
        self.lidar_front_sub = rospy.Subscriber("thorvald_001/front_scan", LaserScan, callback=self.callback_front_lidar)
        self.lidar_back_sub = rospy.Subscriber("thorvald_001/back_scan", LaserScan, callback=self.callback_back_lidar)
        self.front_scan = None
        self.back_scan = None
        self.robot_reference = "/thorvald_001/odom"
    
    def callback_front_lidar(self, scan):
        #print("Scan update")
        self.front_scan = scan

    def callback_back_lidar(self, scan):
        #print("Scan udate back")
        self.back_scan = scan

    def polar_to_cartesian(self, angle, range):
        return Point(math.cos(angle) * range, math.sin(angle) * range,0)

    def get_min_coordinate(self, scan):
        index_max = np.argmin(scan.ranges)
        angle = scan.angle_min + index_max * scan.angle_increment
        range = scan.ranges[index_max]
        return self.polar_to_cartesian(angle, range)

    def get_point_in_odm(self,scan):
        min_point_local = self.get_min_coordinate(scan)
        min_point_local_stamped = PointStamped(header=scan.header,point=min_point_local)
        min_point_odom = self.listener.transformPoint(self.robot_reference, min_point_local_stamped)
        # trans_front, rot_front = self.listener.lookupTransform(self.robot_reference, scan.header.frame_id, rospy.Time())
        # print("trans_front: " + str(trans_front))
        # print("rot_front: " + str(rot_front))
        # print("min_pose.position:" + str(min_pose.position))
        # pos = do_transform_point(min_pose.position,Point(*trans_front))
        return min_point_odom

    def get_pose_from_scan(self):
        if self.back_scan is None or self.front_scan is None:
            return None

        front_point = self.get_point_in_odm(self.front_scan).point
        back_point = self.get_point_in_odm(self.back_scan).point
        print("front point:" + str(front_point))
        front_dist = np.linalg.norm(np.array([front_point.x, front_point.y, front_point.z]))
        back_dist = np.linalg.norm(np.array([back_point.x, back_point.y, back_point.z]))

        min_point =  front_point if front_dist < back_dist else back_point
        min_pose = Pose(position=min_point)
        return min_pose
        

    def run(self):
       while not rospy.is_shutdown():
            try:
                pose = self.get_pose_from_scan()
                print("New pose:" + str(pose))
                self.display_pose(pose)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("Exception :(" + e.message)
                pass
            
            self.rate.sleep()

    def display_pose(self, pose):
        if pose is None:
            return
        p1 = PoseStamped()
        p1.header.frame_id = self.robot_reference
        p1.pose.orientation = pose.orientation
        p1.pose.position = pose.position
        self.pose_pub.publish(p1)
        print("displayed.")

if __name__ == "__main__":
    tf_listener = TFListener()
    tf_listener.run()