#!/usr/bin/env python3
# rosrun object_search simulated_lidar_node.py

import rospy
import math
from sensor_msgs.msg import LaserScan
from dmce_msgs.msg import RobotPosition, RobotMapUpdate


class SimulatedLidarNode:
    def __init__(self):
        rospy.init_node("simulated_lidar_node")

        # 订阅机器人位置与地图更新
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        rospy.Subscriber("/robot1/RobotMapUpdates", RobotMapUpdate, self.map_callback)

        # 发布雷达数据
        self.scan_pub = rospy.Publisher("/object_search/simulated_scan", LaserScan, queue_size=10)

        # 机器人位置
        self.robot_pos = None

        # 雷达参数
        self.angle_min = 0.0
        self.angle_max = 2 * math.pi
        self.angle_increment = math.radians(1.0)  # 1° 分辨率
        self.range_min = 0.05
        self.range_max = 10.0

        rospy.loginfo("✅ Simulated Lidar Node 已启动")

    # ---------------------------
    # 回调函数：机器人位置
    # ---------------------------
    def pos_callback(self, msg: RobotPosition):
        self.robot_pos = (msg.x_position, msg.y_position)

    # ---------------------------
    # 回调函数：地图更新点
    # ---------------------------
    def map_callback(self, msg: RobotMapUpdate):
        if self.robot_pos is None:
            return

        xr, yr = self.robot_pos
        judgment_threshold = 50.0  # 判断阈值
        num_beams = int((self.angle_max - self.angle_min) / self.angle_increment)
        ranges = [self.range_max] * num_beams

        # 遍历所有障碍点，计算相对角度和距离
        for x, y in zip(msg.x_positions, msg.y_positions):
            dx = x - xr
            dy = y - yr
            r = math.hypot(dx, dy)
            if r < self.range_min or r > self.range_max:
                continue

            theta = math.atan2(dy, dx)
            if theta < 0:
                theta += 2 * math.pi  # 转为 0~2π

            # 对应的扫描角索引
            idx = int((theta - self.angle_min) / self.angle_increment)
            if 0 <= idx < num_beams:
                ranges[idx] = min(ranges[idx], r)

        # 构造 LaserScan 消息
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "robot1/robot1"  # ✅ 与 RViz 中 frame 对应

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 10.0  # 10 Hz
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.scan_pub.publish(scan)
        rospy.loginfo_throttle(2.0, f"发布激光帧: {len(ranges)} beams")

    # ---------------------------
    # 主循环
    # ---------------------------
    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = SimulatedLidarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
