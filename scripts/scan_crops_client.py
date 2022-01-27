#! /usr/bin/python
import rospy
import actionlib

from ros_crop_estimation.msg import ScanCropsAction, ScanCropsGoal

class ScanCropsClient:
    
    list_of_waypoints = []
    def __init__(self):
        rospy.init_node('scan_crops_client')

        self.list_of_waypoints = rospy.get_param('topological_path', ["winetasting_0", "winetasting_1"])
        rospy.loginfo("Running crop scan with path:" + str(self.list_of_waypoints))

        self.client = actionlib.SimpleActionClient('scan_crops', ScanCropsAction)
        self.client.wait_for_server()
        self.result = []
    
    def send_goal(self):
        for waypint in self.list_of_waypoints:
            goal = ScanCropsGoal()
            #Fill in the goal here
            goal.crop_waypoint = waypint
            rospy.loginfo("sending new waypoint goal:" + waypint)
            self.client.send_goal(goal)
            status = self.client.wait_for_result(rospy.Duration(30))
            curr_result = self.client.get_result()
            rospy.loginfo("Scancrops stage completed with status: '%s'", status)
            rospy.loginfo("result is %s", curr_result)
            self.result.append(curr_result)
        rospy.loginfo("Scancrop actions done; no waypoints left. Traversed path: '%s'", str(self.list_of_waypoints))
        rospy.loginfo("final result is %s", self.result)

if __name__ == "__main__":
    try:
        scan_crops_client = ScanCropsClient()
        scan_crops_client.send_goal()
        rospy.spin()

    except Exception as e:
        print(e)
        pass