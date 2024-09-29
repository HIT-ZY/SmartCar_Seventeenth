#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

# Author:周毅
# Date:2022.8.9

#####################################################

#####################################################

# 包含库文件
import threading
import time
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
from sensor_msgs.msg import Imu, BatteryState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_srvs.srv import Empty
from xf_mic_asr_offline.srv import Get_Offline_Result_srv
import argparse
import time
import dynamic_reconfigure.client
from pathlib import Path
from photo_and_qr.srv import *

#####################################################

#####################################################
#
# 说明：
# 1.
# 2.
#
#####################################################

#####################################################

# 定义全局变量

# 进入每一房间方案：

# B房间
# 1表示在门口逆时针转1/4圈拍照
# 2表示以进去位姿在房间中间转一整圈拍照
# 3表示以进去位姿在房间中间顺时针转半圈拍照
# 4表示以进去位姿在房间中间逆时针转半圈拍照
# 5表示以正对照片位姿在房间中间顺时针转半圈拍照
prog_B = 1

# C房间
# 1表示在门口逆时针转1/4圈拍照
# 2表示以进去位姿在房间中间转一整圈拍照
# 3表示以进去位姿在房间中间顺时针转半圈拍照
# 4表示以进去位姿在房间中间逆时针转半圈拍照
prog_C = 1

# D房间
# 1表示在门口顺时针转1/4圈拍照
# 2表示以进去位姿在房间中间转一整圈拍照
# 3表示以进去位姿在房间中间顺时针转半圈拍照
# 4表示以进去位姿在房间中间逆时针转半圈拍照
prog_D = 1

# 拍照数目统计
num = 1

# 每个房间拍照数
photo_num = 16

# 旋转拍照转速:
# 顺时针转速
ang_vel_clockwise = 1.5
# 逆时针转速
ang_vel_anticlockwise = 1.5


# 拍照间隔:
# 1/4圈拍照间隔60ms
interval_quarter = 0.06
# 半圈拍照间隔120ms
interval_half = 0.12
# 整圈拍照间隔240ms
interval_complete = 0.24


# 停车保险钥匙，打开设为True，关闭设为False, D区固定目标点时需要置为False
parking_safety_key = True

# 从D区出来后强制停车时间，用于到了停车点failed to get a plan 的处理
shut_down = 6.0

# 图片存放及修改路径
path = "/home/ucar/pic"

# 语音唤醒钥匙
wake_key = 'Nobody is here but us chickens.'       # a = '370'

# 旋转拍照策略，置为0表示四段式拍照，置为1表示不停旋转拍照
rot_flag = 1

#####################################################

#####################################################

# 裁剪图片
def Image_Main():
    file_lst = os.listdir(path)
    for file in file_lst:
        File_Process(path + "/" + file, 0.4)

def File_Process(file_dir, ratio):
    img = cv2.imread(file_dir)
    target = img[150:480,:,:].copy()
    os.remove(file_dir)
    cv2.imwrite(file_dir, target)

# 清除代价地图
def Clear_Costmap():
    rospy.wait_for_service("/move_base/clear_costmaps")
    try:
        greetings_client = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        res = greetings_client.call()
        rospy.loginfo("Clear costmap")
    except rospy.ServiceException as e:
        rospy.logwarn("Clear costmap failed: %s"%e)

# 出弯时抢占目标点
def Seize_Target(move_base, target):
    Set_Target(move_base, target)
    while True:
        loc = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
        loc_x = loc.pose.pose.position.x
        loc_y = loc.pose.pose.position.y
        if loc_x > 5.0 and loc_y < -0.3:
            break

# 启动备用停车点，用于处理D区扫到的停车点卡死的情况
def Aux_Park(move_base, target_E_leftrear, target_E):
    v_flag = 0
    v_begin_time = rospy.Time.now()
    # 设置被困时间限制为2.0秒
    while (rospy.Time.now() - v_begin_time).to_sec() < 2.0:
        vel = rospy.wait_for_message('/odom', Odometry, timeout = None)
        vx = vel.twist.twist.linear.x
        vy = vel.twist.twist.linear.y
        # az = vel.twist.twist.angular.z
        v = math.sqrt((vx ** 2) + (vy ** 2))
        if v > 0.05:
            v_flag = 1
            break
    if v_flag == 0:
        Set_Target(move_base, target_E_leftrear)
        while True:
            state_toEhead = move_base.get_state()
            if state_toEhead == GoalStatus.SUCCEEDED:
                break
        Set_Target(move_base, target_E)

# 从车上的系统时间获取车完赛时间，并记录到桌面上
def Finish_Time(running_time):
    f = open("/home/ucar/Desktop/runningtime.txt", "w")
    f.write(str(running_time)) 
    f.close()

# 获取电量，并记录到桌面上
def Get_Battery():
    res = rospy.wait_for_message('/battery_state', BatteryState, timeout = None)
    battery_level =  res.percentage
    # 把电量写入文件
    f = open("/home/ucar/Desktop/batterylevel.txt", "w")
    f.write(str(battery_level)) 
    f.close()

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
    rospy.loginfo("Have deleted photos!")

# 设定目标点位姿
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

# 拍照
def Take_Photo(sleep_flag):
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
    # 根据拍照停顿时间控制转圈
    if sleep_flag == 0:
        time.sleep(interval_quarter)
    elif sleep_flag == 1:
        time.sleep(interval_half)
    elif sleep_flag == 2:
        time.sleep(interval_complete)

# 多线程拍照
def Multithread_Photo(sleep_flag):
    # sleep_flag为0转1/4圈， 为1转半圈， 为2转整圈
    thread_photo = threading.Thread(target = Take_Photo, args = (sleep_flag,))
    thread_photo.start()
    thread_photo.join()

# 房间门口拍照
def Door_Photo(pub_cmd, whichroom):
    global num 
    while num <= (whichroom * photo_num):
        # B,C房间门口逆时针拍照
        if whichroom == 1 or whichroom == 2:
            Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, ang_vel_anticlockwise)
        # D房间门口顺时针拍照
        elif whichroom == 3:
            Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, ang_vel_clockwise)
        Multithread_Photo(0)
        num += 1

# 房间中心拍照
def Center_Photo(pub_cmd, whichroom, direct, complete):
    global num
    while num <= (whichroom * photo_num):
        # 0为逆时针拍照
        if direct == 0:
            Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, ang_vel_anticlockwise)
        # 1为顺时针拍照
        elif direct == 1:
            Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, ang_vel_clockwise)

        # 0为转半圈
        if complete == 0:
            Multithread_Photo(1)
            num += 1
        # 1为转一整圈
        elif complete == 1:
            Multithread_Photo(2)
            num += 1

# 到达目标点
def Reach_Target(move_base, target):
    Set_Target(move_base, target)
    while True:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            break

# 进入B房间
def Enter_Room_B(pub_cmd, move_base, target_B_center, target_B_out, whichroom_B):
    if prog_B == 1:
        Reach_Target(move_base, target_B_out)
        Door_Photo(pub_cmd, whichroom_B)
    elif prog_B == 2:
        Reach_Target(move_base, target_B_center)
        Center_Photo(pub_cmd, whichroom_B, 0, 1)
    elif prog_B == 3:
        Reach_Target(move_base, target_B_center)
        Center_Photo(pub_cmd, whichroom_B, 1, 0)
    elif prog_B == 4:
        Reach_Target(move_base, target_B_center)
        Center_Photo(pub_cmd, whichroom_B, 0, 0)
    elif prog_B == 5:
        target_B_center = Pose(Point(4.800, -3.800, 0.000),
                  Quaternion(0.000, 0.000, 0.000, 1.000))
        Reach_Target(move_base, target_B_center)
        Center_Photo(pub_cmd, whichroom_B, 1, 0) 

# 进入C房间
def Enter_Room_C(pub_cmd, move_base, target_C_center, target_C_out, whichroom_C):
    if prog_C == 1:
        Reach_Target(move_base, target_C_out)
        Door_Photo(pub_cmd, whichroom_C)
    elif prog_C == 2:
        Reach_Target(move_base, target_C_center)
        Center_Photo(pub_cmd, whichroom_C, 0, 1)
    elif prog_C == 3:
        Reach_Target(move_base, target_C_center)
        Center_Photo(pub_cmd, whichroom_C, 1, 0)
    elif prog_C == 4:
        Reach_Target(move_base, target_C_center)
        Center_Photo(pub_cmd, whichroom_C, 0, 0)
    
# 进入D房间
def Enter_Room_D(pub_cmd, move_base, target_D_center, target_D_out, whichroom_D):
    if prog_D == 1:
        Reach_Target(move_base, target_D_out)
        Clear_Costmap()
        Door_Photo(pub_cmd, whichroom_D)
    elif prog_D == 2:
        Reach_Target(move_base, target_D_center)
        Clear_Costmap()
        Center_Photo(pub_cmd, whichroom_D, 0, 1)
    elif prog_D == 3:
        Reach_Target(move_base, target_D_center)
        Clear_Costmap()
        Center_Photo(pub_cmd, whichroom_D, 1, 0)
    elif prog_D == 4:
        Reach_Target(move_base, target_D_center)
        Clear_Costmap()
        Center_Photo(pub_cmd, whichroom_D, 0, 0)   

# 导航主函数
def Navigation_Main():
    global num
    global prog_B
    global prog_C
    global prog_D
    global photo_num
    global ang_vel_clockwise
    global ang_vel_anticlockwise
    global interval_quarter
    global interval_half
    global interval_complete
    global parking_safety_key
    global wake_key

    # 用于区分三个房间的标志,用于拍照数的限制
    whichroom_B = 1
    whichroom_C = 2
    whichroom_D = 3

    # 摄像头和图像变换参数
    # dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    # newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    # mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    font = cv2.FONT_HERSHEY_SIMPLEX

    # 读取方案传入参数
    prog_B = int(sys.argv[1])
    rospy.loginfo("prog_B: " + str(prog_B))
    prog_C = int(sys.argv[2])
    rospy.loginfo("prog_C: " + str(prog_C))
    prog_D = int(sys.argv[3])
    rospy.loginfo("prog_D: " + str(prog_D))

    # 删除上一次的照片
    Delete_Photos()

    # 设置目标点坐标和朝向
    target_bendout = Pose(Point(5.500, -1.000, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))

    target_B_center = Pose(Point(4.800, -3.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_B_out = Pose(Point(4.300, -2.900, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))

    target_C_center = Pose(Point(3.050, -4.600, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_C_out = Pose(Point(3.000, -3.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))

    target_D_center = Pose(Point(1.250, -4.000, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_D_out = Pose(Point(1.600, -2.800, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))
    target_D_strategy = Pose(Point(1.250, -5.500, 0.000),
                  Quaternion(0.000, 0.000, -0.707, 0.707))

    target_E = Pose(Point(0.080, -1.060, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.000)) 
    target_E_leftrear = Pose(Point(0.600, -1.500, 0.000),
                  Quaternion(0.000, 0.000, 0.924, 0.382))
    target_E_back = Pose(Point(0.300, -1.080, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.000))  
           
    # 声明move_base动作库客户端，等待5s连接
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server!")

    # 已唤醒
    if wake_key != 'Jesus':            # a != '370'
        # 唤醒后清空代价地图
        Clear_Costmap()
        rospy.sleep(1.0)
        
        # 设置计算完赛时间的开始时间
        start_time = rospy.Time.now()

        # 向其他节点发布话题
        # 发布速度                    
        pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        # pub_room = rospy.Publisher('enter_room', Int32, queue_size = 4)
        # 唤醒yolo
        pub_yolo = rospy.Publisher('yolo_wake', Int32, queue_size = 4)
        # 异步拍照，改成多线程
        # pub_photo = rospy.Publisher('photo', Int32, queue_size = 10)
        # 动态调参标志
        pub_terminal = rospy.Publisher('reach_target', Int32, queue_size = 1)
        rospy.loginfo("Publishers have been established!")
        
        # 出弯后的抢占目标点
        Seize_Target(move_base, target_bendout)
        Enter_Room_B(pub_cmd, move_base, target_B_center, target_B_out, whichroom_B)
        Enter_Room_C(pub_cmd, move_base, target_C_center, target_C_out, whichroom_C)
        Enter_Room_D(pub_cmd, move_base, target_D_center, target_D_out, whichroom_D)

        # 向停车区进发，发送停车区动态调参标志符
        pub_terminal.publish(1)

        # 终点E
        Set_Target(move_base, target_E)

        if parking_safety_key == True:
            Aux_Park(move_base, target_E_leftrear, target_E)

        rospy.loginfo("Check!")
        # 设置5分钟时间限制
        # finished_within_time = move_base.wait_for_result(rospy.Duration(300))
        time_flag = 1
        while True:
            if time_flag == 1:
                site = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
                siteY = site.pose.pose.position.y
                if siteY > -2.5:
                    # 设置到时间直接停车头时间戳(解决到停车点failed to get a plan问题)
                    head_time =  rospy.Time.now()
                    time_flag = 0
            if time_flag == 0:
                tail_time = rospy.Time.now()
                last_time = (tail_time - head_time).to_sec()
            # 最大容忍时间
            if time_flag == 0 and last_time > shut_down:
                move_base.cancel_goal()
                Set_Target(move_base, target_E_back)
                rospy.loginfo("Head to target_E_add!")
                while True:
                    state = move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        break
                break
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            # if not finished_within_time:
            #     break
        # while True:
        #     state = move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         break
        print("Attention please, last_time is coming!")
        print(last_time)

        # 辅助停车
        Client_Srv()

        end_time = rospy.Time.now()
        # 计算小车运行总时间
        running_time = end_time - start_time
        running_time = running_time.secs
        rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        # 处理图片
        Image_main()
        # 识别
        pub_yolo.publish(room_flag)
        # 记录时间
        Finish_Time(running_time)
        # 获取电量
        Get_Battery()
        # 杀死节点
        Kill_All_Nodes()

# # 原地旋转然后拍照，每转90度拍一张，每个房间拍4张
# def Single_Rotation_Photo(num, pub, set_orz, az):
#     # 每次转到位置拍两张
#     Take_Photo(num)
#     num += 1
#     Take_Photo(num)
#     num += 1
#     while True:
#         # result = rospy.wait_for_message('/odom', Odometry, timeout = None)
#         result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
#         orz = result.pose.pose.orientation.z
#         Twist_Pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, az)
#         if orz >= set_orz:
#             break
#     return num

# # 原地旋转拍照一周，每隔45度左右拍一张照片，每个房间拍8张左右
# def Rotate_Ceaselessly_Photo(num, pub_cmd, pub_photo, az, every_time, position):
#     init_num = num
#     # 初始朝向拍一张
#     # Take_Photo(num)
#     pub_photo.publish(position)
#     num += 1
#     flag = 1
#     begin_time = rospy.Time.now()
#     while (num - init_num) <= 7:
#         Twist_Pub(pub_cmd, 0.0, 0.0, 0.0, 0.0, 0.0, az)
#         if int((rospy.Time.now() - begin_time).to_sec() / every_time) == flag:
#             pub_photo.publish(position)
#             num += 1
#             flag += 1
#     return num

# 位置坐标检测（调试用）
def Position_Test():
    result_amcl = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
    pos_x_amcl = result_amcl.pose.pose.position.x
    pos_y_amcl = result_amcl.pose.pose.position.y
    rospy.loginfo("Position from amcl: " + str(pos_x_amcl) + " " + str(pos_y_amcl))

    result_odom = rospy.wait_for_message('/odom', Odometry, timeout = None)
    pos_x_odom = result_odom.pose.pose.position.x
    pos_y_odom = result_odom.pose.pose.position.y
    rospy.loginfo("Position from odom: " + str(pos_x_odom) + " " + str(pos_y_odom))
 
# 姿态朝向检测（调试用）
def Quaternion_test(num):
    result = rospy.wait_for_message('/odom', Odometry, timeout = None)
    # result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
    orz = result.pose.pose.orientation.z
    rospy.loginfo(str(num) + ": " + str(orz))

def Listener():
    # 在函数中声明全局变量以便可以修改它
    global wake_key
    # 初始化move_test节点
    rospy.init_node('move_test', anonymous = True)
    # 等待麦克风唤醒
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout = None)
    Navigation_Main()

if __name__ == "__main__":
    Listener()

#####################################################
