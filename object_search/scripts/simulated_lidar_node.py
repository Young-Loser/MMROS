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

        # 雷达参数（按波束数计算，避免角度端点重复）
        self.angle_min = 0.0
        self.angle_increment = math.radians(1.0)  # 1° 分辨率
        # 计算波束数并确定 angle_max 使得 angle_max = angle_min + (num_beams-1)*angle_increment
        self.num_beams = int(round(2 * math.pi / self.angle_increment))
        self.angle_max = self.angle_min + (self.num_beams - 1) * self.angle_increment

        self.range_min = 0.05
        self.range_max = 10.1

        rospy.loginfo("✅ Simulated Lidar Node 已启动")

    # ---------------------------
    # 回调函数：机器人位置
    # ---------------------------
    def pos_callback(self, msg: RobotPosition):
        self.robot_pos = (msg.x_position, msg.y_position)

    # ---------------------------
    # 回调函数：地图更新点（已使用 values 字段）
    # ---------------------------
    def map_callback(self, msg: RobotMapUpdate):
        if self.robot_pos is None:
            return

        xr, yr = self.robot_pos
        num_beams = self.num_beams
        judgment_threshold = 50.0  # 判断阈值
        # 初始为最大距离
        ranges = [self.range_max] * num_beams
        # 激光强度/置信度数组，对应每个波束
        intensities = [0.0] * num_beams

        # 遍历所有点，使用 values 作为强度/置信度（values==0.0 视为无障碍或不可信）
        # 使用 zip 对应 x,y,values（若长度不一致，zip 会按最短裁减）
        for x, y, v in zip(msg.x_positions, msg.y_positions, msg.values):
            # 跳过无效或置信度为零的点
            if v is None:
                continue
            try:
                value = float(v)
            except Exception:
                # 非数值则跳过
                continue
            if value <= judgment_threshold:
                continue

            dx = x - xr
            dy = y - yr
            r = math.hypot(dx, dy)
            if r < self.range_min or r > self.range_max:
                continue

            theta = math.atan2(dy, dx)
            if theta < 0:
                theta += 2 * math.pi  # 转为 0~2π

            # 计算索引并分配到相邻两个波束
            idx_float = (theta - self.angle_min) / self.angle_increment
            idx0 = int(math.floor(idx_float)) % num_beams
            idx1 = (idx0 + 1) % num_beams

            # 使用最近距离优先，同时记录对应的 value（强度/置信度）
            if r < ranges[idx0]:
                ranges[idx0] = r
                intensities[idx0] = value
            elif r == ranges[idx0]:
                # 若距离相同，保留更大置信度
                intensities[idx0] = max(intensities[idx0], value)

            if r < ranges[idx1]:
                ranges[idx1] = r
                intensities[idx1] = value
            elif r == ranges[idx1]:
                intensities[idx1] = max(intensities[idx1], value)

        # 构造 LaserScan 消息
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "robot1/robot1"  # ✅ 与 RViz 中 frame 对应

        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / 20  # 20 Hz
        # 更合理地设置 time_increment
        scan.time_increment = scan.scan_time / float(num_beams) if num_beams > 0 else 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        # 将 values 映射到 intensities（保留原始置信度）
        scan.intensities = intensities

        self.scan_pub.publish(scan)
        rospy.loginfo_throttle(2.0, f"发布激光帧: {len(ranges)} beams (used points with value>0)")

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
