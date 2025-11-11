#!/usr/bin/env python3
# rosrun object_search mapupdate_to_cloud.py
import rospy
import numpy as np
from dmce_msgs.msg import RobotMapUpdate
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

def callback(msg):
    points = []
    for x, y, v in zip(msg.x_positions, msg.y_positions, msg.values):
        if v > 0.1:  # 占用点
            points.append([x, y, 0.0])  # z=0

    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "map"
    # header.frame_id = f"robot{msg.robotId}/robot{msg.robotId}"

    cloud = pc2.create_cloud_xyz32(header, points)
    pub.publish(cloud)

if __name__ == "__main__":
    rospy.init_node("mapupdate_to_cloud")
    pub = rospy.Publisher("/robot1/map_pointcloud", PointCloud2, queue_size=1)
    rospy.Subscriber("/robot1/RobotMapUpdates", RobotMapUpdate, callback)
    rospy.spin()

