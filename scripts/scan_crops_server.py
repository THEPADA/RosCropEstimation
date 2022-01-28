#! /usr/bin/python2
import rospy
import actionlib
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from ros_crop_estimation.msg import ScanCropsAction, ScanCropsResult
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class ScanCropLinesServer:
    """
    A server to manage crop line scanning actions.
    It automatically navigates to the specified waypoints and returns the count of grape-bunches.
    """
    def __init__(self):
        rospy.init_node('scan_cropline_server')
        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()

        # first start server to prevent race condition
        self.server = actionlib.SimpleActionServer('scan_crops', ScanCropsAction, self.execute, False)
        self.server.start()
        self.scaned_crop_lines = 0
        self.scanned_bunches = 0


        
    def execute(self, scan_crop_goal):
        # Do lots of robot stuff here
        self.client.wait_for_server()

        # send first goal
        goal = GotoNodeGoal()
        goal.target = scan_crop_goal.crop_waypoint
        self.client.send_goal(goal)
        status = self.client.wait_for_result(rospy.Duration(120)) # wait until the action is complete
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)


        rospy.loginfo("Waiting for global object map to check crop estimate")
        pc_global_obj = rospy.wait_for_message("/object_detector/global_map", PointCloud2)
        object_points = point_cloud2.read_points(pc_global_obj)
        self.scanned_bunches = len(list(object_points))
        self.scaned_crop_lines += 1
        ret_result = ScanCropsResult()
        ret_result.crop_estimated = self.scanned_bunches
        self.server.set_succeeded(ret_result)

if __name__ == '__main__':
    server = ScanCropLinesServer()
    rospy.spin()