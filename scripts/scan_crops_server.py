#! /usr/bin/python2
import rospy
import actionlib

from ros_crop_estimation.msg import ScanCropLineAction, ScanCropLineResult

class ScanCropLinesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('scan_crops', ScanCropLineAction, self.execute, False)
        self.server.start()
        self.scaned_crop_lines = 0

    def execute(self, goal):
        # Do lots of robot stuff here
        # total dishes count is incremented each time the action is executed, and returned as result
        self.scaned_crop_lines += 1
        result = ScanCropLineResult()
        result.scaned_crop_lines = self.scaned_crop_lines
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('scan_cropline_server')
    server = ScanCropLinesServer()
    rospy.spin()