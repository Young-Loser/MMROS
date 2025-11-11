#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检测近距离点云中的连续线段
"""
# rosrun object_search object_detector_node.py

import rospy
import os
import numpy as np
import yaml
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from dmce_msgs.msg import RobotPosition

class ObjectDetectorNode:
    def __init__(self):
        rospy.init_node("object_detector_node", anonymous=True)
        rospy.loginfo("📡 启动物体检测节点（3m范围内线段检测 + 物体匹配）")

        # 参数
        self.radius = rospy.get_param("~radius", 3.0)
        self.min_segment_points = rospy.get_param("~min_segment_points", 5)
        self.max_point_gap = rospy.get_param("~max_point_gap", 0.2)
        self.safe_distance = rospy.get_param("~safe_distance", 0.1)

        # 状态
        self.robot_pos = np.array([0.0, 0.0])
        self.latest_cloud = None

        # 加载 YAML 文件
        yaml_path = os.path.join(os.path.dirname(__file__), "../apartment1_all.yaml")
        yaml_path = os.path.abspath(yaml_path)
        if not os.path.exists(yaml_path):
            rospy.logerr(f"❌ 找不到 YAML 文件: {yaml_path}")
            exit(1)

        with open(yaml_path, "r") as f:
            self.scene_data = yaml.safe_load(f)
        rospy.loginfo(f"✅ 成功加载场景文件: {yaml_path}")

        # ROS接口
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        rospy.Subscriber("/robot1/map_pointcloud", PointCloud2, self.cloud_callback)
        self.segment_pub = rospy.Publisher("/object_search/detected_segments", PointCloud2, queue_size=1)

        rospy.loginfo("✅ 已订阅 /robot1/RobotPosition 与 /robot1/map_pointcloud")
        self.rate = rospy.Rate(5)
        self.run()

    # ---------------- 回调函数 ----------------
    def pos_callback(self, msg):
        self.robot_pos = np.array([msg.x_position, msg.y_position])

    def cloud_callback(self, msg):
        """接收点云并筛选半径范围内的点"""
        pts = np.array([[p[0], p[1]] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)])
        if pts.size == 0:
            return

        d = np.linalg.norm(pts - self.robot_pos, axis=1)
        pts = pts[d < self.radius]
        self.latest_cloud = pts

    # ---------------- 核心算法 ----------------
    def detect_segments(self, pts):
        """简单连续线段检测算法（保持原逻辑）"""
        if len(pts) == 0:
            return []

        rel = pts - self.robot_pos
        angles = np.arctan2(rel[:, 1], rel[:, 0])
        order = np.argsort(angles)
        pts_sorted = pts[order]

        segments, current_seg = [], [pts_sorted[0]]
        for i in range(1, len(pts_sorted)):
            dist = np.linalg.norm(pts_sorted[i] - pts_sorted[i - 1])
            if dist < self.max_point_gap:
                current_seg.append(pts_sorted[i])
            else:
                if len(current_seg) >= self.min_segment_points:
                    segments.append(np.array(current_seg))
                current_seg = [pts_sorted[i]]

        if len(current_seg) >= self.min_segment_points:
            segments.append(np.array(current_seg))

        return segments

    # ---------------- 改进后的物体匹配 ----------------
    def find_object_by_segment(self, seg):
        """判断整个线段是否与某个物体重叠（带容错距离）"""
        if "objects" not in self.scene_data:
            return None

        tolerance = rospy.get_param("~match_tolerance", 0.1)  # ✅ 默认容差0.1m
        min_hits = rospy.get_param("~min_object_hits", 2)     # ✅ 至少命中2个点

        for obj in self.scene_data["objects"]:
            if "ros" not in obj or "center" not in obj["ros"]:
                continue

            center = obj["ros"]["center"]
            size = obj["ros"]["size"]

            # 带容错的矩形范围
            xmin = center["x"] - size["width"] / 2 - tolerance
            xmax = center["x"] + size["width"] / 2 + tolerance
            ymin = center["y"] - size["height"] / 2 - tolerance
            ymax = center["y"] + size["height"] / 2 + tolerance

            # 统计线段中有多少个点落在物体范围内
            inside = np.logical_and.reduce([
                seg[:, 0] >= xmin,
                seg[:, 0] <= xmax,
                seg[:, 1] >= ymin,
                seg[:, 1] <= ymax
            ])

            hit_count = np.sum(inside)
            if hit_count >= min_hits:
                return obj.get("name", "unknown")

        return None


    # ---------------- 发布结果 ----------------
    def publish_segments(self, pts):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]
        cloud = pc2.create_cloud(header, fields, np.c_[pts, np.zeros(len(pts))])
        self.segment_pub.publish(cloud)

    # ---------------- 主循环 ----------------
    def run(self):
        while not rospy.is_shutdown():
            if self.latest_cloud is None or len(self.latest_cloud) == 0:
                self.rate.sleep()
                continue

            segments = self.detect_segments(self.latest_cloud)
            if len(segments) == 0:
                self.rate.sleep()
                continue

            # rospy.loginfo_throttle(2.0, f"✅ 检测到 {len(segments)} 条线段")

            # 合并点云以可视化
            combined = np.vstack(segments)
            self.publish_segments(combined)

            # 检测匹配的物体
            detected_objects = set()

            for seg in segments:
                cx, cy = np.mean(seg[:, 0]), np.mean(seg[:, 1])
                name = self.find_object_by_segment(seg)
                if name:
                    detected_objects.add(name)

            # 打印所有检测到的物体（单行显示）
            if detected_objects:
                print("当前检测到物体：", ", ".join(sorted(detected_objects)))

            self.rate.sleep()


if __name__ == "__main__":
    try:
        ObjectDetectorNode()
    except rospy.ROSInterruptException:
        pass
    
