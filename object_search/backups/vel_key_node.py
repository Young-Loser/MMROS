#!/usr/bin/env python3
# coding=utf-8

import rospy
from math import pi
from geometry_msgs.msg import Twist
from pynput import keyboard


adv_key_msg = [0, 0, 0, 0]  # 分别对应wasd四个键的状态


def fun_on_press(key):
    """定义按下时候的响应,参数传入key"""
    try:
        flag = key.char
        # str_flag=str(key)
        # flag=str_flag[1]
        if flag == 'w':
            adv_key_msg[0] = 1
        elif flag == 'a':
            adv_key_msg[1] = 1
        elif flag == 's':
            adv_key_msg[2] = 1
        elif flag == 'd':
            adv_key_msg[3] = 1
    except AttributeError:
        rospy.logwarn('On_AttributeError')


def fun_on_release(key):
    """定义释放时候的响应"""
    try:
        flag = key.char
        # str_flag=str(key)
        # flag=str_flag[1]
        if flag == 'w':
            adv_key_msg[0] = 0
        elif flag == 'a':
            adv_key_msg[1] = 0
        elif flag == 's':
            adv_key_msg[2] = 0
        elif flag == 'd':
            adv_key_msg[3] = 0
    except AttributeError:
        rospy.logwarn('off_AttributeError')


def listen_key_nblock():
    listener = keyboard.Listener(
        on_press=fun_on_press, on_release=fun_on_release)
    listener.start()  # 启动线程


if __name__ == "__main__":
    rospy.init_node('vel_key_node')
    rospy.logwarn('hua_ti_kai_shi')
    listen_key_nblock()
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)

    # 开始时发步的命令
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0

    # vel_msg_1 = Twist()
    # vel_msg_1.linear.x = 0
    # vel_msg_1.linear.y = 0
    # vel_msg_1.angular.z= 0

    while not rospy.is_shutdown():
        ##ws控制前后，ad控制左右
        ##四个键位不会互相冲突，可以同时摁住wa，机器人往左前方行走
        ##如果同时摁住ws，或者ad，控制变量会相互抵消，机器人不会移动
        vel_msg.linear.x = (adv_key_msg[0]-adv_key_msg[2])*0.5
        vel_msg.angular.z = (adv_key_msg[1]-adv_key_msg[3])*10*pi/30
        # rospy.loginfo('vel_key_node->')
        # print('vel_key_node->', adv_key_msg)
        vel_pub.publish(vel_msg)
        rate.sleep()
