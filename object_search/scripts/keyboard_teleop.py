#!/usr/bin/env python3
# rosrun object_search keyboard_teleop.py

import rospy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from dmce_msgs.msg import RobotPosition   # ✅ 自定义消息

# 全向移动 + 旋转控制
MOVE_BINDINGS = {
    'd': (1.0, 0.0, 0.0),    # 前进
    'a': (-1.0, 0.0, 0.0),   # 后退
    'w': (0.0, 1.0, 0.0),    # 左移
    's': (0.0, -1.0, 0.0),   # 右移
    'q': (0.0, 0.0, 1.0),    # 左转
    'e': (0.0, 0.0, -1.0),   # 右转
}

EXIT_KEY = 'x'  # 🔑 一键退出改成 X，避免和右转 e 冲突

speed = 3   # 线速度 (m/s)
turn  = 1.0   # 角速度 (rad/s)

class KeyboardControlNode:
    def __init__(self):
        rospy.init_node("keyboard_control_node")
        self.cmd_pub = rospy.Publisher("/robot1/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/robot1/RobotPosition", RobotPosition, self.pos_callback)
        self.current_pos = None

        self.settings = termios.tcgetattr(sys.stdin)
        self.twist = Twist()

    def pos_callback(self, msg):
        """订阅位置并打印"""
        self.current_pos = (msg.x_position, msg.y_position)
        print(f"[位置] x = {msg.x_position:6.2f} | y = {msg.y_position:6.2f}", end="\r")

    def get_key(self):
        """读取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        rospy.loginfo("键盘控制启动：WASD 前后左右移动，Q/E 旋转，X 退出")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            key = self.get_key()

            if key == EXIT_KEY:
                rospy.loginfo("用户按下退出键，节点关闭")
                rospy.signal_shutdown("用户退出")
                break

            if key in MOVE_BINDINGS:
                x, y, th = MOVE_BINDINGS[key]
                self.twist.linear.x = x * speed
                self.twist.linear.y = y * speed
                self.twist.angular.z = th * turn
            else:
                # 松开键自动停止
                self.twist.linear.x = 0.0
                self.twist.linear.y = 0.0
                self.twist.angular.z = 0.0

            self.cmd_pub.publish(self.twist)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = KeyboardControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

