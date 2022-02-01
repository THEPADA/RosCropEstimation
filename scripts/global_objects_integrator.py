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
        self.global_objects = np.array([], dtype=object)
        self.global_objects.shape = (-1, 4)
        self.freeze_objects = False
        print("init done")

    def from_local_objects_detected(self, local_objects_pc):
        """
        This function is called when a new point cloud is received from the local object detector.
        @param local_objects_pc: The point cloud of the objects detected by the local object detector.
        @type local_objects_pc: PointCloud2
        @return: None
        """

        local_object_positions_and_classes = np.array(point_cloud2.read_points_list(local_objects_pc))
        local_object_positions_and_classes.shape = (-1, 4)
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.UINT32, 1),
                  #PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        header = rospy.Header()
        header.frame_id = "map"
        header.stamp = local_objects_pc.header.stamp

        # extract position from local positions and classes
        self.add_new_detected_points(local_object_positions_and_classes)

        object_classes = self.global_objects[:, -1]
        number_of_unique_classes = len(set(object_classes))
        color_values = sns.color_palette("viridis", number_of_unique_classes).as_hex()
        class_color_dict = dict(zip(set(object_classes), color_values))
        
        global_points = np.copy(self.global_objects)

        global_points[:,-1] = np.array([self.hex_to_rgb_packed(class_color_dict[cls]) for cls in global_points[:,-1]])

        point_cloud = point_cloud2.create_cloud(
            header, fields, global_points)

        self.pub_global_objects.publish(point_cloud)
        
    def hex_to_rgb_packed(self, hex):
        """
        Convert hex to rgb.
        @param hex: Hex color code.
        @type hex: String
        @return: RGB color code.
        """
        hex = hex.lstrip('#')
        hlen = len(hex)
        r,g,b = tuple(int(hex[i:i+hlen//3], 16) for i in range(0, hlen, hlen//3))
        return ((r&0x0ff)<<16)|((g&0x0ff)<<8)|(b&0x0ff)

    def add_new_detected_points(self, new_points):
        """
        add new points to the global map.
        @param new_points: The points to add to the global map.
        @type new_points: numpy array (n,4) for the x, y, z coorindates and class_id of an object
        @return: None
        """
        rospy.loginfo_throttle(5, "prior points" +
                               str(len(self.global_objects)))
        # print("<dist>")
        for new_point in new_points:
            if len(filter(lambda x: not(np.isnan(x)), new_point)) != 4: 
                continue
            if new_point[2] < 0.1:
                continue
            
            if self.get_close_point_indices(new_point[:3], self.global_objects[:,:3]) > 0.40:
                
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
