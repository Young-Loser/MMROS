#!/usr/bin/env python3
# rosrun object_search lidar_node_test.py

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from dmce_msgs.msg import RobotPosition, RobotMapUpdate

class SimulatedLidarNode:
    def __init__(self):
        rospy.init_node("simulated_lidar_node")

        # # 订阅机器人位置与地图更新
        # rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        # rospy.Subscriber("/robot1/RobotMapUpdates", RobotMapUpdate, self.map_callback)

        # 发布器
        self.scan_pub = rospy.Publisher("/object_search/simulated_scan", LaserScan, queue_size=10)

        # 机器人位置
        self.robot_pos = None

        # === 激光参数 ===
        # self.frame_id = "robot1/robot1"    # 🔹 RViz 中 Fixed Frame 对应
        self.frame_id = "robot1/robot1"    # 🔹 RViz 中 Fixed Frame 对应
        self.range_max = 10              # 半径 10 m
        self.range_min = 0.05
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(1.0)   # 每 1° 一束光（共 360 条）

        rospy.loginfo("✅ 模拟雷达启动，范围 = %.1f m，共 %d 个射线",
                      self.range_max,
                      int((self.angle_max - self.angle_min) / self.angle_increment))
    

    def publish_circle(self):
        """发布一个完整的圆形激光扫描"""
        scan = LaserScan()
        scan.header = Header()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = self.frame_id

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        num_beams = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        # 每个方向固定距离 -> 形成一个圆
        scan.ranges = [ 9.9 for _ in range(num_beams)]
        scan.intensities = []

        self.scan_pub.publish(scan)
        rospy.loginfo_throttle(2.0, f"发布激光帧: {len(scan.ranges)} beams")

    def run(self):
        rate = rospy.Rate(10)   # 每秒 5 次
        while not rospy.is_shutdown():
            self.publish_circle()
            rate.sleep()


if __name__ == "__main__":
    try:
        node = SimulatedLidarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass