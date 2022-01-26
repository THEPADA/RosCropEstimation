import rospy
import actionlib

from uol_cmp9767m_tutorial.msg import DoDishesAction, DoDishesGoal

class ScanCropsClient:
    def __init__(self):
        rospy.init_node('scan_crops_client')
        self.client = actionlib.SimpleActionClient('scan_crop', DoDishesAction)
        self.client.wait_for_server()
    
    def send_goal(self, target):
        goal = DoDishesGoal()
        #Fill in the goal here
        self.client.send_goal(goal)
        goal.target = target
        status = self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)