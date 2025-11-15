#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# rosrun object_search vel_key_node.py

"""
WASD 键盘控制节点 (支持同时按键)
适用于 ROS 中的全向或仿真机器人
Topic: /robot1/cmd_velrosrun object_search vel_key_node.py
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard
import numpy as np
from dmce_msgs.msg import RobotPosition
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class KeyboardTeleopNode:
    def __init__(self):
        rospy.init_node("keyboard_teleop_node_safe")
        rospy.loginfo("✅ 键盘控制节点启动 (带避障 + 多机器人切换)")

        # ------------ 多机器人支持 ------------
        self.nRobots = rospy.get_param("~nRobots", 3)
        self.current_robot = 1  # 当前控制机器人（默认 robot1）
        rospy.loginfo(f"🎮 当前控制 robot{self.current_robot}")

        # 话题发布器和订阅器
        self.cmd_pubs = {}
        self.pos = {i: None for i in range(1, self.nRobots + 1)}

        for i in range(1, self.nRobots + 1):
            self.cmd_pubs[i] = rospy.Publisher(f"/robot{i}/cmd_vel", Twist, queue_size=10)
            rospy.Subscriber(f"/robot{i}/RobotPosition", RobotPosition, self.pos_callback, callback_args=i)

        rospy.Subscriber("/robot1/map_pointcloud", PointCloud2, self.cloud_callback)

        # ------------ 参数 ------------
        self.linear_speed = rospy.get_param("~linear_speed", 2.0)
        self.safe_distance = rospy.get_param("~safe_distance", 0.15)
        self.rate = rospy.Rate(30)

        # 状态变量
        self.closest_obstacle = {
            "left": float("inf"), "right": float("inf"),
            "front": float("inf"), "back": float("inf")
        }
        self.adv_key_msg = [0, 0, 0, 0]  # d s a w ← 你之前修改过的方向

        # ------------ 键盘监听 ------------
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    # =====================================================
    # 回调函数
    # =====================================================
    def pos_callback(self, msg, rid):
        self.pos[rid] = np.array([msg.x_position, msg.y_position])

    def cloud_callback(self, msg):
        """只用于当前机器人避障"""
        rid = self.current_robot
        if self.pos[rid] is None:
            return

        pts = np.array([[p[0], p[1]] for p in pc2.read_points(msg, ("x", "y"), skip_nans=True)])
        if pts.size == 0:
            return

        dx, dy = pts[:, 0] - self.pos[rid][0], pts[:, 1] - self.pos[rid][1]
        dist = np.hypot(dx, dy)

        self.closest_obstacle["front"] = np.min(dist[(dy > 0) & (abs(dx) < 0.3)]) \
            if np.any((dy > 0) & (abs(dx) < 0.3)) else float("inf")
        self.closest_obstacle["back"] = np.min(dist[(dy < 0) & (abs(dx) < 0.3)]) \
            if np.any((dy < 0) & (abs(dx) < 0.3)) else float("inf")
        self.closest_obstacle["left"] = np.min(dist[(dx < 0) & (abs(dy) < 0.3)]) \
            if np.any((dx < 0) & (abs(dy) < 0.3)) else float("inf")
        self.closest_obstacle["right"] = np.min(dist[(dx > 0) & (abs(dy) < 0.3)]) \
            if np.any((dx > 0) & (abs(dy) < 0.3)) else float("inf")

    # =====================================================
    # 键盘事件
    # =====================================================
    def on_press(self, key):
        """按键按下"""
        try:
            flag = key.char.lower()

            # -------- 机器人切换：按 1 2 3 4 --------
            if flag.isdigit():
                rid = int(flag)
                if 1 <= rid <= self.nRobots:
                    self.current_robot = rid
                    rospy.loginfo(f"🎮 切换至 robot{self.current_robot}")
                return

            # -------- WASD --------
            if flag == "w":  # 前
                self.adv_key_msg[3] = 1
            elif flag == "a":  # 左
                self.adv_key_msg[2] = 1
            elif flag == "s":  # 下
                self.adv_key_msg[1] = 1
            elif flag == "d":  # 右
                self.adv_key_msg[0] = 1

        except AttributeError:
            pass

    def on_release(self, key):
        """按键松开"""
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

    # =====================================================
    # 主循环
    # =====================================================
    def run(self):
        twist = Twist()

        while not rospy.is_shutdown():
            rid = self.current_robot  # 当前机器人

            vx = (self.adv_key_msg[0] - self.adv_key_msg[2]) * self.linear_speed
            vy = (self.adv_key_msg[3] - self.adv_key_msg[1]) * self.linear_speed

            # 只对当前机器人做避障
            if self.closest_obstacle["right"] < self.safe_distance and vx > 0:
                vx = 0
                rospy.logwarn_throttle(1.0, f"robot{rid} → 🚫 右侧障碍太近")
            if self.closest_obstacle["left"] < self.safe_distance and vx < 0:
                vx = 0
                rospy.logwarn_throttle(1.0, f"robot{rid} → 🚫 左侧障碍太近")
            if self.closest_obstacle["front"] < self.safe_distance and vy > 0:
                vy = 0
                rospy.logwarn_throttle(1.0, f"robot{rid} → 🚫 前方障碍太近")
            if self.closest_obstacle["back"] < self.safe_distance and vy < 0:
                vy = 0
                rospy.logwarn_throttle(1.0, f"robot{rid} → 🚫 后方障碍太近")

            # 发布控制
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = 0
            self.cmd_pubs[rid].publish(twist)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        node = KeyboardTeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

