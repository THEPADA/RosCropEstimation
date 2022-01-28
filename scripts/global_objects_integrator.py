#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import seaborn as sns

import matplotlib.pyplot as plt


class GlobalObjectIntegrator:
    """
    This class takes the world coordinates form the object dected by the robot and integrates them with
    the already known object coordinates. 
    """

    def __init__(self):
        print("init called")
        rospy.init_node("global_object_integrator", log_level=rospy.INFO)
        self.sub_local_objects_detected = rospy.Subscriber(
            "/object_detector/objects_pointcloud", PointCloud2, callback=self.from_local_objects_detected)
        self.pub_global_objects = rospy.Publisher(
            "/object_detector/global_map", PointCloud2, queue_size=10)
        print("creating members + defaults")
        self.global_objects = np.array([], dtype=float)
        self.global_objects.shape = (-1, 3)
        self.freeze_objects = False
        print("init done")

    def from_local_objects_detected(self, local_objects_pc):
        """
        This function is called when a new point cloud is received from the local object detector.
        @param local_objects_pc: The point cloud of the objects detected by the local object detector.
        @type local_objects_pc: PointCloud2
        @return: None
        """

        if self.freeze_objects:
            return

        local_points = point_cloud2.read_points_list(local_objects_pc)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  # PointField('rgb', 12, PointField.UINT32, 1),
                  #PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        header = rospy.Header()
        header.frame_id = "map"
        header.stamp = local_objects_pc.header.stamp

        self.add_new_detected_points(local_points)
        point_cloud = point_cloud2.create_cloud(
            header, fields, self.global_objects)

        self.pub_global_objects.publish(point_cloud)

    def add_new_detected_points(self, new_points):
        """
        add new points to the global map.
        @param new_points: The points to add to the global map.
        @type new_points: numpy array of points.
        @return: None
        """
        rospy.loginfo_throttle(5, "prior points" +
                               str(len(self.global_objects)))
        # print("<dist>")
        for new_point in new_points:
            if len(filter(lambda x: not(np.isnan(x)), new_point)) != 3: continue
            if new_point[2] < 0.1:
                continue
            if self.get_close_point_indices(new_point, self.global_objects) > 0.40:
                self.global_objects = np.vstack(
                    (self.global_objects, new_point))
        #        print("added points")
        # print("</dist>")
        rospy.loginfo_throttle(5, "new points" + str(len(new_points)))
        rospy.loginfo_throttle(5, "posterior points" +
                               str(len(self.global_objects)))

    def get_close_point_indices(self, node, nodes):
        """
        This function returns the indices of the points in the nodes array that are close to the node.
        @param node: The point to check for close points.
        @type node: numpy array of points.
        @param nodes: The array of points to check for close points.
        @type nodes: numpy array of points.
        @return: The indices of the points in the nodes array that are close to the node.
        @rtype: numpy array of indices.
        """
        if len(nodes) == 0:
            return np.inf
        nodes = np.asfarray(nodes)
        dist_2 = np.sqrt(np.sum((nodes - node)**2, axis=1))
        # print(dist_2)
        return dist_2[np.argmin(dist_2)]


if __name__ == "__main__":
    try:
        GlobalObjectIntegrator()
        rospy.spin()

    except Exception as e:
        print(e)
        pass
