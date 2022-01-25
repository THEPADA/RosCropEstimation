#! /usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class RobotController:
    def __init__(self):
        rospy.init_node("MotorControl")
        self.lidar_subsriber = rospy.Subscriber("/thorvald_001/front_scan", callback=self.callback_scan, data_class=LaserScan)
        self.imu_publisher = rospy.Publisher("/thorvald_001/twist_mux/cmd_vel", data_class=Twist)

    def callback_scan(self, msg):

        min_range = np.min(msg.ranges)

        twistUpdate = Twist()
        if min_range < 3:
            twistUpdate.angular.z = np.random.rand()
        else:
            twistUpdate.linear.x = 1.0
        self.imu_publisher.publish(twistUpdate)
        
if __name__ == "__main__":
    try:
        RobotController()
        rospy.spin()
    except e as Error:
        pass