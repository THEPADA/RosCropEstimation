#! /usr/bin/python
from syslog import LOG_INFO
import rospy
import actionlib
import traceback
import sys

from ros_crop_estimation.msg import ScanCropsAction, ScanCropsGoal


class ScanCropsClient:
    """
    This is the client that handles actions for scanning crops.
    """

    list_of_waypoints = []

    def __init__(self):
        """
        Initialize the client.
        """
        rospy.init_node('scan_crops_client', log_level=rospy.INFO)

        self.list_of_waypoints = rospy.get_param(
            '~topological_path', ["winetasting_0", "winetasting_1"])
        rospy.loginfo("Running crop scan with path:" +
                      str(self.list_of_waypoints))

        self.client = actionlib.SimpleActionClient(
            'scan_crops', ScanCropsAction)
        self.client.wait_for_server()
        self.result = []

    def send_goal(self):
        """
        Send a new goal to the server.
        The goal contains the list of waypoints to visit.
        The server will navigate the robot to the waypoints.
        On success, the server will return the number of bunches.
        """
        self.client.wait_for_server()
        rospy.sleep(5)
        print(self.list_of_waypoints)
        for waypint in self.list_of_waypoints:
            goal = ScanCropsGoal()
            # Fill in the goal here
            goal.crop_waypoint = waypint
            print("sending new waypoint goal:" + waypint)
            self.client.send_goal(goal)
            status = self.client.wait_for_result(rospy.Duration(120))
            curr_result = self.client.get_result()
            print("Scancrops stage completed with status: '%s'", status)
            print("result is %s", curr_result)
            self.result.append(curr_result)
        rospy.loginfo("Scancrop actions done; no waypoints left. Traversed path: '%s'", str(
            self.list_of_waypoints))
        rospy.loginfo("final result is %s", self.result)


if __name__ == "__main__":
    try:
        scan_crops_client = ScanCropsClient()
        scan_crops_client.send_goal()
        rospy.spin()
    except Exception as e:
        print(traceback.format_exc())
