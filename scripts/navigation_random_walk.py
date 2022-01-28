#! /usr/bin/python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class RobotController:
    """
    This is an alternative navigation metodology based on random walking.
    """

    def __init__(self):
        """
        Initialize the robot controller.
        @return: None
        """
        rospy.init_node("MotorControl")
        self.lidar_subsriber = rospy.Subscriber(
            "/thorvald_001/front_scan", callback=self.callback_scan, data_class=LaserScan)
        self.imu_publisher = rospy.Publisher(
            "/thorvald_001/twist_mux/cmd_vel", data_class=Twist)

    def callback_scan(self, msg):
        """
        Use the lidar sensor to navigate the robot.
        @param msg: The lidar scan message.
        @type msg: LaserScan
        @return: None
        """

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
    except Exception as e:
        print(e)
        pass
