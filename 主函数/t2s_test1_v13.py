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
from nav_msgs.msg  import Odometry
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
from photo_and_qr.srv import *

#####################################################

#####################################################

# 定义全局变量

# 语音唤醒钥匙
wake_key = 'Nobody is here but us chickens.'       # a = '370'

# 旋转拍照策略，置为0表示四段式拍照，置为1表示不停旋转拍照
rot_flag = 1

# 选择进入那两个房间，选择进入B,C房间置为0，B,D房间置为1，C,D房间置为2
room_flag = 0

#####################################################

#####################################################

# 二维码辅助停车客户端
def Client_Srv():
    rospy.wait_for_service("greetings")
    try:
        greetings_client = rospy.ServiceProxy("greetings", Greeting)
        res = greetings_client.call(1)
        rospy.loginfo("Server call: %s"%res.feedback)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s"%e)
    # return res.feedback

# 清空拍照文件夹
def Delete_Photos():
    # 删除文件夹
    shutil.rmtree("/home/ucar/pic")
    # 重新创建文件夹
    os.mkdir("/home/ucar/pic")
    # 分三个文件夹分别存放三个房间的图片
    #shutil.rmtree("/home/ucar/pic/Room_B")
    #os.mkdir("/home/ucar/pic/Room_B")
    #shutil.rmtree("/home/ucar/pic/Room_C")
    #os.mkdir("/home/ucar/pic/Room_C")
    #shutil.rmtree("/home/ucar/pic/Room_D")
    #os.mkdir("/home/ucar/pic/Room_D")
    rospy.loginfo("Have deleted photos!")

# 拍照
def Take_Photo(num):
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("Picture-%s", str(num))
    # 设定图片尺寸，格式
    width = 640
    height = 480
    cap.set(3, width)
    cap.set(4, height)
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    ret, frame = cap.read()
    # cv2.waitKey(20)
    cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
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
    global room_flag
    num = 1
    # 不间断拍照的传输标志，用于把照片写进不同的文件夹
    position_B = Int32(1)
    position_C = Int32(2)
    position_D = Int32(3)
    # 摄像头和图像变换参数
    # dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    # newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    # mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    # 删除上一次的照片
    Delete_Photos()
    # 设置目标点坐标和朝向
    target_B = Pose(Point(5.000, -4.250, 0.000),
                  Quaternion(0.000, 0.000, 0.000, 1.000))
    target_B_out = Pose(Point(4.500, -2.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))              
    target_C = Pose(Point(3.250, -4.750, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_C_out = Pose(Point(3.000, -3.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_D = Pose(Point(1.250, -4.250, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_D_out = Pose(Point(2.500, -2.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_E = Pose(Point(0.100, -1.100, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.000)) # 0.13
    target_E_head = Pose(Point(0.600, -1.500, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.000))
    # 声明move_base动作库客户端，等待5s连接
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server!")
    # 已唤醒
    if wake_key != 'Jesus':            # a != '370'
        start_time = rospy.Time.now() 
        # 向其他节点发布话题                    
        pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        pub_bobao = rospy.Publisher('bobao', Int32, queue_size = 1)
        pub_yolo = rospy.Publisher('yolo_wake', Int32, queue_size = 4)
        pub_photo = rospy.Publisher('photo', Int32, queue_size = 8)
        pub_terminal = rospy.Publisher('reach_target', Int32, queue_size = 1)
        rospy.loginfo("Publishers have been established!")
        # # B, C, D三个房间
        # num = Goto_Target(num, target_B, move_base, pub_cmd, pub_photo, position_B)
        # num = Goto_Target(num, target_C, move_base, pub_cmd, pub_photo, position_C)
        # num = Goto_Target(num, target_D, move_base, pub_cmd, pub_photo, position_D)
        # 进入B,C房间识别
        if room_flag == 0:
            num = Goto_Target(num, target_B, move_base, pub_cmd, pub_photo, position_B)
            num = Goto_Target(num, target_C, move_base, pub_cmd, pub_photo, position_C)
            Set_Target(move_base, target_D_out)
            while True:
                state = move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
        # 进入B,D房间识别
        elif room_flag == 1:
            num = Goto_Target(num, target_B, move_base, pub_cmd, pub_photo, position_B)
            Set_Target(move_base, target_C_out)
            while True:
                state = move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
            num = Goto_Target(num, target_D, move_base, pub_cmd, pub_photo, position_D)
        # 进入C,D房间识别
        elif room_flag == 2:
            Set_Target(move_base, target_B_out)
            while True:
                state = move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    break
            num = Goto_Target(num, target_C, move_base, pub_cmd, pub_photo, position_C)
            num = Goto_Target(num, target_D, move_base, pub_cmd, pub_photo, position_D)
        else:
            rospy.loginfo("Room selection failed!")
        # 向停车区进发，发送停车区动态调参标志符
        pub_terminal.publish(1)
        # 终点E
        Set_Target(move_base, target_E)
        # state_toE = move_base.get_state()
        # if state_toE == GoalStatus.LOST:
        #     # ABORTED
        #     # REJECTED
        #     Set_Target(move_base, target_E_head)
        #     while True:
        #         state_toEhead = move_base.get_state()
        #         if state_toEhead == GoalStatus.SUCCEEDED:
        #             break
        #     Set_Target(move_base, target_E)
        rospy.loginfo("Check!")
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
        # 播报成功停车之前辅助停车
        # auxpark_begin_time = rospy.Time.now()
        # while (rospy.Time.now() - auxpark_begin_time).to_sec() <= 5.0
            # feedback = Client_Srv()
            # if feedback == "Finished!":
                # break
        Client_Srv()
        if state == GoalStatus.SUCCEEDED:
            pub_bobao.publish(wake_key)
            pub_yolo.publish(Int32(room_flag))
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        else:
            rospy.loginfo("Failed!")
        Kill_All_Nodes()

def Listener():
    # 在函数中声明全局变量以便可以修改它
    global wake_key
    rospy.init_node('move_test', anonymous = True)
    # 等待麦克风唤醒
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout = None)
    Navigation_Main()
    # rospy.wait_for_service("greetings")
    # try:
    #     move_test = rospy.ServiceProxy("greetings", Greeting)
    #     res = move_test.call(1)
    #     rospy.loginfo("Server call: %s"%res.feedback)
    # except rospy.ServiceException as e:
    #     rospy.logwarn("Service call failed: %s"%e)
    # Kill_All_Nodes()

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
def Goto_Target(num, target, move_base, pub_cmd, pub_photo, position):
    global rot_flag
    Set_Target(move_base, target)
    while True:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            break
    # 此时已经到达目标点
    if rot_flag == 0:
        # 第三个参数为姿态变换参数，第四个参数为转速
        # AMCL
        # # Quaternion_test(num)
        # num = Single_Rotation_Photo(num, pub_cmd, -0.5, 1.0)
        # # Quaternion_test(num)
        # num = Single_Rotation_Photo(num, pub_cmd, 0.5, 1.0)
        # # Quaternion_test(num)
        # num = Single_Rotation_Photo(num, pub_cmd, 0.95, 1.0)
        # # Quaternion_test(num)
        # ODOM
        # Quaternion_test(num)
        num = Single_Rotation_Photo(num, pub_cmd, 0.0, 0.5)
        # Quaternion_test(num)
        num = Single_Rotation_Photo(num, pub_cmd, 0.7, 0.5)
        # Quaternion_test(num)
        num = Single_Rotation_Photo(num, pub_cmd, 0.9, 0.5)
        # Quaternion_test(num)
        # 最后一个位置拍照
        Take_Photo(num)
        num += 1
        Take_Photo(num)
        num += 1

    elif rot_flag == 1:
        # 第四个参数为转速，第五个参数为拍照时间间隔
        num = Rotate_Ceaselessly_Photo(num, pub_cmd, pub_photo, 1.0, 0.85, position)
    else:
        rospy.loginfo("An error occurred in the rotation photographing strategy!")
    return num

# 原地旋转然后拍照，每转90度拍一张，每个房间拍4张
def Single_Rotation_Photo(num, pub, set_orz, az):
    # 每次转到位置拍两张
    Take_Photo(num)
    num += 1
    Take_Photo(num)
    num += 1
    while True:
        result = rospy.wait_for_message('/odom', Odometry, timeout = None)
        # result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
        orz = result.pose.pose.orientation.z
        Twist_Pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, az)
        if orz >= set_orz:
            break
    return num

# 原地旋转拍照一周，每隔45度左右拍一张照片，每个房间拍8张左右
def Rotate_Ceaselessly_Photo(num, pub_cmd, pub_photo, az, every_time, position):
    init_num = num
    # 初始朝向拍一张
    # Take_Photo(num)
    pub_photo.publish(position)
    num += 1
    flag = 1
    begin_time = rospy.Time.now()
    while (num - init_num) <= 7:
        Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, az)
        if int((rospy.Time.now() - begin_time).to_sec() / every_time) == flag:
            pub_photo.publish(position)
            num += 1
            flag += 1
    return num

# 姿态朝向检测（调试用）
def Quaternion_test(num):
    result = rospy.wait_for_message('/odom', Odometry, timeout = None)
    # result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
    orz = result.pose.pose.orientation.z
    rospy.loginfo(str(num) + ": " + str(orz))

if __name__ == "__main__":
    Listener()

#####################################################
