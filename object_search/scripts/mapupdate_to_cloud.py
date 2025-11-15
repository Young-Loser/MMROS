#!/usr/bin/env python3
# rosrun object_search mapupdate_to_cloud.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from dmce_msgs.msg import RobotMapUpdate
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


def make_callback(pub):
    """为每个机器人生成独立 callback（使用闭包绑定 pub）"""
    def callback(msg):
        points = []
        for x, y, v in zip(msg.x_positions, msg.y_positions, msg.values):
            if v > 0.1:  # 只取占用点
                points.append([x, y, 0.0])

        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        cloud = pc2.create_cloud_xyz32(header, points)
        pub.publish(cloud)
    return callback


if __name__ == "__main__":
    rospy.init_node("mapupdate_to_cloud")

    # -----------------------------
    # 读取 launch 文件中的 nRobots
    # -----------------------------
    nRobots = rospy.get_param("~nRobots", 1)
    rospy.loginfo(f"[mapupdate_to_cloud] 启动，检测到 nRobots = {nRobots}")

    pubs = []

    # ----------------------------------------
    # 为每个 robotX 创建 subscriber & publisher
    # ----------------------------------------
    for rid in range(1, nRobots + 1):
        topic_sub = f"/robot{rid}/RobotMapUpdates"
        topic_pub = f"/robot{rid}/map_pointcloud"

        pub = rospy.Publisher(topic_pub, PointCloud2, queue_size=1)
        pubs.append(pub)

        rospy.Subscriber(topic_sub, RobotMapUpdate, make_callback(pub))

        rospy.loginfo(f"[mapupdate_to_cloud] 订阅 {topic_sub}")
        rospy.loginfo(f"[mapupdate_to_cloud] 发布 {topic_pub}")

    rospy.spin()

