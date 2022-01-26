import rospy
import actionlib

from ws02_robot_control.msg import ScanCropLineAction, ScanCropLineResult

class ScanCropLinesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('scan_crops', ScanCropLineAction, self.execute, False)
        self.server.start()
        self.scaned_crop_lines = 0

    def execute(self, goal):
        # Do lots of robot stuff here
        # total dishes count is incremented each time the action is executed, and returned as result
        self.total_dishes_cleaned += 1
        result = ScanCropLineResult()
        result.total_dishes_cleaned = self.total_dishes_cleaned
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('scan_cropline_server')
    server = ScanCropLinesServer()
    rospy.spin()