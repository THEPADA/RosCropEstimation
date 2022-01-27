#!/usr/bin/python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np

class GlobalObjectIntegrator:
    def __init__(self):
        print("init called")
        rospy.init_node("global_object_integrator", log_level=rospy.INFO)
        self.sub_local_objects_detected = rospy.Subscriber("/object_detector/objects_pointcloud", PointCloud2, callback=self.from_local_objects_detected)
        self.pub_global_objects = rospy.Publisher("/object_detector/global_map",PointCloud2, queue_size=10)
        print("creating members + defaults")
        self.global_objects = np.array([],dtype=float)
        self.global_objects.shape = (-1,3)
        self.freeze_objects = False
        print("init done")

    def from_local_objects_detected(self, local_objects_pc):

        if self.freeze_objects: return 

        local_points = point_cloud2.read_points_list(local_objects_pc)
        self.add_new_detected_points(self.global_objects)
        
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            # PointField('rgb', 12, PointField.UINT32, 1),
            #PointField('rgba', 12, PointField.UINT32, 1),
        ]

        header = rospy.Header()
        header.frame_id = "map"

        self.add_new_detected_points(local_points)
        point_cloud = point_cloud2.create_cloud(header, fields, self.global_objects)

        self.pub_global_objects.publish(point_cloud)

    def add_new_detected_points(self, new_points):
        rospy.loginfo_throttle(5, "prior points" + str(len(self.global_objects)))
        for new_point in new_points:
            if len(filter(lambda x: not(np.isnan(x)), new_point)) != 3: continue
            if self.get_close_point_indices(new_point, self.global_objects) > 0.02:
                self.global_objects = np.vstack((self.global_objects, new_point))

        rospy.loginfo_throttle(5,"new points" + str(len(new_points)))
        rospy.loginfo_throttle(5,"posterior points" + str(len(self.global_objects)))

    def get_close_point_indices(self, node, nodes):
        if len(nodes) == 0: return np.inf
        nodes = np.asfarray(nodes)
        dist_2 = np.sum((nodes - node)**2, axis=1)
        #print(dist_2)
        return dist_2[np.argmin(dist_2)]

if __name__ == "__main__":
    try:
        GlobalObjectIntegrator()
        rospy.spin()

    except Exception as e:
        print(e)
        pass

