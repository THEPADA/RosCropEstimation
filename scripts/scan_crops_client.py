#! /usr/bin/python2
import rospy
import actionlib

from ros_crop_estimation.msg import ScanCropAction, ScanLineGoal

class ScanCropsClient:
    def __init__(self):
        rospy.init_node('scan_crops_client')

        self.list_of_waypoints = rospy.get_param('topological_path', [])

        self.client = actionlib.SimpleActionClient('scan_crops', ScanCropAction)
        self.client.wait_for_server()
        self.result = []
    
    def send_goal(self):
        for waypint in self.list_of_waypoints:
            goal = ScanLineGoal()
            #Fill in the goal here
            self.client.send_goal(goal)
            goal.target = waypint
            status = self.client.wait_for_result(rospy.Duration.from_sec(5.0))
            curr_result = self.client.get_result()
            rospy.loginfo("Scancrops stage completed with status: '%s'", status)
            rospy.loginfo("result is %s", curr_result)
            self.result.append(curr_result)
        rospy.loginfo("Scancrop actions done; no waypoints left. Traversed path: '%s'", str(self.list_of_waypoints))
        rospy.loginfo("final result is %s", self.result)
            