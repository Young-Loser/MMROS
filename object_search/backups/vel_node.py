#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
import csv
from math import pi
from geometry_msgs.msg import Twist
from pynput import keyboard
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


work_flag = 0# 对应当前工作状态 0表示无工作 1表示走八字 2表示键盘控制运动
adv_key_msg = [0, 0, 0, 0]  # 分别对应wasd四个键的状态
vel_angularz = pi/6 #机器人转向速度
imu_log_data = []#记录imu数据



def fun_on_press(key):
    """定义按下时候的响应,参数传入key"""
    global work_flag
    try:
        flag = key.char
        # str_flag=str(key)
        # flag=str_flag[1]
        # print(flag,type(flag))
        if flag == '1':
            work_flag = 1
        elif flag == '2':
            work_flag = 2
        elif flag == 'w':
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
    global work_flag
    try:
        flag = key.char
        # str_flag=str(key)
        # flag=str_flag[1]
        if flag == '1':
            pass
        elif flag == '2':
            pass
        elif flag == 'w':
            adv_key_msg[0] = 0
        elif flag == 'a':
            adv_key_msg[1] = 0
        elif flag == 's':
            adv_key_msg[2] = 0
        elif flag == 'd':
            adv_key_msg[3] = 0
    except AttributeError:
        rospy.logwarn('off_AttributeError')

#启动键盘监听线程
def listen_key_nblock():
    listener = keyboard.Listener(
        on_press=fun_on_press, on_release=fun_on_release)
    listener.start()  # 启动线程

flag_imu=1
# IMU 回调函数
def imu_callback(msg):
    global vel_angularz
    global flag_imu
    global imu_log_data
    if msg.orientation_covariance[0] < 0:
        return
    # 四元数转成欧拉角
    quaternion = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    ]
    (roll,pitch,yaw) = euler_from_quaternion(quaternion)
    # 弧度换算成角度
    roll = roll*180/math.pi
    pitch = pitch*180/math.pi
    yaw = yaw*180/math.pi
    # 根据imu数据 使机器人走八字轨迹
    if 179 < yaw < 180 and flag_imu==1 :
        vel_angularz = -vel_angularz
        flag_imu=0
    elif -5 < yaw < 5:
        flag_imu=1
    rospy.loginfo("Roll= %.2f Pitch= %.2f Yaw= %.2f", roll, pitch, yaw)
    ##存数据到csv文件中
        # 将新数据添加到列表中
    new_row = [roll,pitch,yaw]
    # 将列表写入 CSV 文件的新行
    writer.writerow(new_row)
    # 刷新缓冲区，确保数据被写入文件
    file.flush()

if __name__ == "__main__":
    rospy.init_node('vel_key_node')
    rospy.logwarn('The topic begins')
    listen_key_nblock()
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    imu_sub = rospy.Subscriber("/imu/data",Imu,imu_callback,queue_size=10)
    rate = rospy.Rate(30)

    # 开始时发步的命令
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.angular.z = 0
    # 指定要保存的文件路径和文件名
    filename = 'imu_data.csv'
    file = open(filename, 'a', newline='')
    ##清空log
    file.truncate(0)
    writer = csv.writer(file)
    while not rospy.is_shutdown():
        ##ws控制前后，ad控制左右
        ##四个键位不会互相冲突，可以同时摁住wa，机器人往左前方行走
        ##如果同时摁住ws，或者ad，控制变量会相互抵消，机器人不会移动
        if work_flag == 1:
            vel_msg.linear.x = 0.5
            vel_msg.angular.z = vel_angularz
        if work_flag==2:
            flag_imu=1
            vel_msg.linear.x = (adv_key_msg[0]-adv_key_msg[2])*0.5
            vel_msg.angular.z = (adv_key_msg[1]-adv_key_msg[3])*10*pi/30
        # print('vel_angularz',vel_angularz)    
        # rospy.loginfo('vel_key_node->')
        # print('vel_key_node->', adv_key_msg)
        vel_pub.publish(vel_msg)
        rate.sleep()
