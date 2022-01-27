#! /usr/bin/python2
import rospy
import actionlib

from ros_crop_estimation.msg import ScanCropsAction, ScanCropsResult
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class ScanCropLinesServer:
    def __init__(self):
        # first start server to prevent race condition
        rospy.init_node('scan_cropline_server')
        self.server = actionlib.SimpleActionServer('scan_crops', ScanCropsAction, self.execute, False)
        self.server.start()
        self.scaned_crop_lines = 0

        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
        
    def execute(self, scan_crop_goal):
        # Do lots of robot stuff here
        
        # send first goal
        goal = GotoNodeGoal()
        goal.target = scan_crop_goal.crop_waypoint
        self.client.send_goal(goal)
        status = self.client.wait_for_result(rospy.Duration(30)) # wait until the action is complete
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)
        self.scaned_crop_lines += 1
        ret_result = ScanCropsResult()
        ret_result.crop_estimated = self.scaned_crop_lines
        self.server.set_succeeded(ret_result)

if __name__ == '__main__':
    server = ScanCropLinesServer()
    rospy.spin()