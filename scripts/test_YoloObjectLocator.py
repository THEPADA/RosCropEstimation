#!/usr/bin/python3
import unittest

from ws02_robot_control.scripts.YoloObjectLocator import YoloObjectLocator
import YoloObjectLocator


class YoloObjectLocatorTest(unittest.TestCase):
    uut = YoloObjectLocator()


    def verify_format_pointcloud(self, points):
        for point in points:
           self.assertEqual(len(point), 3)
           self.assertTrue(all([type(a) is float for a in point]))

    def verify_format_pointcloud_accepts(self):
        self.verify_format_pointcloud([[0.0,0.0,0.1]])

    @unittest.expectedFailure
    def verify_format_pointcloud_fails(self):
        self.verify_format_pointcloud([[0.0,0.0,0.1]])

if __name__ == '__main__':
    unittest.main()