#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# rosrun object_search vel_key_node.py

"""
WASD 键盘控制节点 (支持同时按键)
适用于 ROS 中的全向或仿真机器人
Topic: /robot1/cmd_velrosrun object_search vel_key_node.py
"""

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard # type: ignore
import numpy as np
from geometry_msgs.msg import Twist
from dmce_msgs.msg import RobotPosition
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class KeyboardTeleopNode:
    def __init__(self):
        rospy.init_node("keyboard_teleop_node_safe")
        rospy.loginfo("✅ 键盘控制节点启动 (带避障功能)")

        # 发布器
        self.pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)

        # 订阅当前位置与点云
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        rospy.Subscriber("/robot1/map_pointcloud", PointCloud2, self.cloud_callback)

        # 参数设置
        self.linear_speed = rospy.get_param("~linear_speed", 2.0)
        self.safe_distance = rospy.get_param("~safe_distance", 0.15)
        self.rate = rospy.Rate(30)

        # 状态变量
        self.pos = None
        self.closest_obstacle = {"left": float("inf"), "right": float("inf"), "front": float("inf"), "back": float("inf")}
        self.adv_key_msg = [0, 0, 0, 0]  # w, a, s, d

        # 键盘监听
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    # ============ 回调函数 ============

    def pos_callback(self, msg):
        """更新机器人位置"""
        self.pos = np.array([msg.x_position, msg.y_position])
        print(f"[位置] x = {msg.x_position:6.2f} | y = {msg.y_position:6.2f}", end="\r")

    def cloud_callback(self, msg):
        """根据点云数据更新最近障碍距离"""
        if self.pos is None:
            return

        pts = np.array([[p[0], p[1]] for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)])
        if pts.size == 0:
            return

        dx = pts[:, 0] - self.pos[0]
        dy = pts[:, 1] - self.pos[1]
        dist = np.hypot(dx, dy)

        # 定义4个区域
        self.closest_obstacle["front"] = np.min(dist[(dy > 0) & (abs(dx) < 0.3)]) if np.any((dy > 0) & (abs(dx) < 0.3)) else float("inf")
        self.closest_obstacle["back"]  = np.min(dist[(dy < 0) & (abs(dx) < 0.3)]) if np.any((dy < 0) & (abs(dx) < 0.3)) else float("inf")
        self.closest_obstacle["left"]  = np.min(dist[(dx < 0) & (abs(dy) < 0.3)]) if np.any((dx < 0) & (abs(dy) < 0.3)) else float("inf")
        self.closest_obstacle["right"] = np.min(dist[(dx > 0) & (abs(dy) < 0.3)]) if np.any((dx > 0) & (abs(dy) < 0.3)) else float("inf")

    # ============ 键盘事件 ============

    def on_press(self, key):
        """键盘按下"""
        try:
            flag = key.char.lower()
            if flag == "w":
                self.adv_key_msg[3] = 1
            elif flag == "a":
                self.adv_key_msg[2] = 1
            elif flag == "s":
                self.adv_key_msg[1] = 1
            elif flag == "d":
                self.adv_key_msg[0] = 1
        except AttributeError:
            pass

    def on_release(self, key):
        """键盘松开"""
        try:
            flag = key.char.lower()
            if flag == "w":
                self.adv_key_msg[3] = 0
            elif flag == "a":
                self.adv_key_msg[2] = 0
            elif flag == "s":
                self.adv_key_msg[1] = 0
            elif flag == "d":
                self.adv_key_msg[0] = 0
        except AttributeError:
            pass

    # ============ 主循环 ============

    def run(self):
        twist = Twist()

        while not rospy.is_shutdown():
            # 默认速度指令
            vx = (self.adv_key_msg[0] - self.adv_key_msg[2]) * self.linear_speed
            vy = (self.adv_key_msg[3] - self.adv_key_msg[1]) * self.linear_speed

            # 根据障碍距离抑制运动
            if self.closest_obstacle["right"] < self.safe_distance and vx > 0:
                vx = 0
                rospy.logwarn_throttle(1.0, "🚫 右侧障碍太近，禁止右移 (vx > 0)")
            if self.closest_obstacle["left"] < self.safe_distance and vx < 0:
                vx = 0
                rospy.logwarn_throttle(1.0, "🚫 左侧障碍太近，禁止左移 (vx < 0)")
            if self.closest_obstacle["front"] < self.safe_distance and vy > 0:
                vy = 0
                rospy.logwarn_throttle(1.0, "🚫 上方障碍太近，禁止前进 (vy > 0)")
            if self.closest_obstacle["back"] < self.safe_distance and vy < 0:
                vy = 0
                rospy.logwarn_throttle(1.0, "🚫 下方障碍太近，禁止后退 (vy < 0)")

            # 发布控制
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = KeyboardTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
