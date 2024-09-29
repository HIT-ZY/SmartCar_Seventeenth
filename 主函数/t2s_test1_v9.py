#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

# Author:周毅
# Date:2022.7.7

#####################################################

#####################################################

# 包含库文件
import numpy as np
import os
import sys
import cv2
import time
import roslib
import rospy
import actionlib
import math
import rosnode
import shutil
from playsound import playsound
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32
from std_msgs.msg import String
from xf_mic_asr_offline.srv import Get_Offline_Result_srv
import argparse
import time
import dynamic_reconfigure.client
from pathlib import Path

#####################################################

#####################################################

# 定义全局变量
# 语音唤醒钥匙
wake_key = 'Nobody is here but us chickens.'       # a = '370'

#####################################################

#####################################################

# 清空拍照文件夹
def Delete_Photos():
    # 删除文件夹
    shutil.rmtree("/home/ucar/pic/Room_B")
    # 重新创建文件夹
    os.mkdir("/home/ucar/pic/Room_B")
    shutil.rmtree("/home/ucar/pic/Room_C")
    os.mkdir("/home/ucar/pic/Room_C")
    shutil.rmtree("/home/ucar/pic/Room_D")
    os.mkdir("/home/ucar/pic/Room_D")
    rospy.loginfo("Have deleted photos!")

# 拍照
def Take_Photo(num):
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("Picture-%s", str(num))
    # 设定图片尺寸，格式
    width = 640
    height = 480
    cap.set(3,  width)
    cap.set(4,  height)
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    ret, frame = cap.read()
    cv2.waitKey(20)
    cv2.imwrite("/home/ucar/pic/" + str(num) + ".jpg", frame)
    cap.release()
    cv2.destroyAllWindows()

# 设定目标点
def Set_Target(move_base, target):
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target))
    move_base.send_goal(goal)

# 发布速度
def Twist_Pub(pub, lx, ly, lz, ax, ay, az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    pub.publish(twist)

# 导航主函数
def Navigation_Main():
    global wake_key 
    num = 1
    Position_B = Int32(1)
    Position_C = Int32(2)
    Position_D = Int32(3)
    # dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    # newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    # mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    Delete_Photos()
    # 设置目标点坐标和朝向
    target_B = Pose(Point(5.000, -4.250, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_C = Pose(Point(3.250, -4.750, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_D = Pose(Point(1.250, -4.250, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_E = Pose(Point(0.130, -1.100, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.000))
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server")
    if wake_key != 'Jesus':            # a != '370'
        start_time = rospy.Time.now()                     
        pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        pub_bobao = rospy.Publisher('bobao', Int32, queue_size = 1)
        pub_yolo = rospy.Publisher('yolo_wake', Int32, queue_size = 4)
        pub_photo = rospy.Publisher('photo', Int32, queue_size = 8)
        pub_terminal = rospy.Publisher('reach_target', Int32, queue_size = 1)
        rospy.loginfo("OK")
        # B, C, D三个房间
        num = Goto_Target(num, target_B, move_base, pub_cmd, pub_photo, Position_B)
        num = Goto_Target(num, target_C, move_base, pub_cmd, pub_photo, Position_C)
        num = Goto_Target(num, target_D, move_base, pub_cmd, pub_photo, Position_D)
        pub_terminal.publish(1)
        # 终点E
        Set_Target(move_base, target_E)
        rospy.loginfo("Check")
        # 设置5分钟时间限制
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))
        while True:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            if not finished_within_time:
                break
        end_time = rospy.Time.now()
        # 计算小车运行总时间
        running_time = end_time - start_time
        running_time = running_time.secs
        if state == GoalStatus.SUCCEEDED:
            pub_bobao.publish(wake_key)
            pub_yolo.publish(wake_key)
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        else:
            rospy.loginfo("Failed!")
        Kill_All_Nodes()

def Listener():
    # 在函数中声明全局变量以便可以修改它
    global wake_key
    rospy.init_node('move_test', anonymous = True)
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout = None)
    Navigation_Main()

# 关闭节点，杀死进程
def Kill_All_Nodes():
    rosnode.kill_nodes(['/ydlidar_node'])
    rosnode.kill_nodes(['/xf_asr_offline_node'])
    rosnode.kill_nodes(['/amcl'])
    rosnode.kill_nodes(['/base_driver'])
    rosnode.kill_nodes(['/dynamic_client'])
    rosnode.kill_nodes(['/dynamic_server'])
    rosnode.kill_nodes(['/map_server'])
    rosnode.kill_nodes(['/move_base'])
    rosnode.kill_nodes(['/robot_pose_ekf'])
    rosnode.kill_nodes(['/sensor_tf_server'])

# 行至目标点拍照
def Goto_Target(num, target, move_base, pub_cmd, pub_photo, Position):
    Set_Target(move_base, target)
    while True:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            break
    # 此时已经到达目标点
    num = Rotate_Ceaselessly(num, pub_cmd, pub_photo, 1.0, 0.7, Position)
    # Quaternion_test(num)
    # num = Rotation_And_Photo(num, pub, -0.1, 0.5)
    # Quaternion_test(num)
    # num = Rotation_And_Photo(num, pub, 0.6, 0.5)
    # Quaternion_test(num)
    # num = Rotation_And_Photo(num, pub, 0.95, 0.5)
    # Quaternion_test(num)
    # Take_Photo(num)
    #num += 1
    return num

# # 原地旋转然后拍照，每转90度拍一张，每个房间拍4张
# def Rotation_And_Photo(num, pub, set_orz, az):
#     Take_Photo(num)
#     while True:
#         result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
#         orz = result.pose.pose.orientation.z
#         Twist_Pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, az)
#         if orz >= set_orz:
#             break
#     num += 1
#     return num

# 原地旋转拍照一周，每隔45度左右拍一张照片，每个房间拍8张左右
def Rotate_Ceaselessly(num, pub_cmd, pub_photo, az, every_time, Position):
    '''Take_Photo(num)
    # begin_time = rospy.Time.now()
    while num<8:
        begin_time = rospy.Time.now()
        while (rospy.Time.now() - begin_time).secs < during_time:
            Twist_Pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, az)
        finish_time = rospy.Time.now()
        running_time = finish_time - begin_time
        running_time = running_time.secs
	num += 1
        Take_Photo(num)
        # rospy.sleep(0.7)
    return num'''
    init_num = num
    # Take_Photo(num)
    pub_photo.publish(Position)
    num += 1
    flag = 1
    begin_time = rospy.Time.now()
    while (num - init_num) <= 7:
        Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, az)
        if int((rospy.Time.now() - begin_time).to_sec() / every_time) == flag:
            pub_photo.publish(Position)
            num += 1
            flag += 1
    return num

# 姿态朝向检测（调试用）
def Quaternion_test(num):
    result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
    orz = result.pose.pose.orientation.z
    rospy.loginfo(str(num)+": "+str(orz))

if __name__ == "__main__":
    Listener()

#####################################################
