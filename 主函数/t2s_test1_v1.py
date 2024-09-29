#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

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
from playsound import playsound
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
# Pose:包括Point position和Quaternion orientation
# Point:以三个float类型的数代表位置坐标
# Quaternion:以四元数(4个float类型的数)代表朝向
# Twist:包括linear(线速度)和angular(角速度)
# PoseWithCovarianceStamped:带有时间标签和参考坐标的估计位姿，包括std_msgs/Header header和pose
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

# 定义全局变量wake_key, wake_key为麦克风的唤醒角度
wake_key='Nobody is here but us chickens.'       # a = '370'

#####################################################

#####################################################

# 拍照，设置图片格式和一些参数
def take_photo(num):
    # 打开车的内置摄像头，读取摄像头
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("take photo-%s", str(num)
    weight = 640
    height = 480
    # 设置分辨率 3和4 分别代表摄像头的属性值。你可以使用函数 cap.get(propId) 来获得视频的一些参数信息。
    # 这里propId 可以是 0 到 18 之间的任何整数。 每一个数代表视频的一个属性,见表其中的一些值可以使用
    # cap.set(propId,value) 来修改, value 就是你想要设置成的新值。例如,我可以使用 cap.get(3) 和 cap.get(4)
    # 来查看每一帧的宽和高。默认情况下得到的值是 640X480。但是我可以使用 ret=cap.set(3,320)和
    # ret=cap.set(4,240) 来把宽和高改成 320X240。
    cap.set(3,  weight)
    cap.set(4,  height)
    # 指定视频编码格式为MJPG
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    print(codec)
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    # cap.read()按帧读取视频，ret,frame是获cap.read()方法的两个返回值。
    #     其中ret是布尔值，如果读取帧是正确的则返回True，如果文件读取到结尾，
    #     它的返回值就为False。frame就是每一帧的图像，是个三维矩阵。
    ret, frame=cap.read()
    # 把第三张图片裁减一下，对于新赛道，当拍的比较准时，可以用此操作来节约算力
    # if num==3:
    #     frame = frame[30:360,40:480].copy()
    # 延时20ms切换到下一帧图像
    cv2.waitKey(20)
    # cv2.imwrite()保存图⽚，共3个参数，第⼀个为保存⽂件名，第⼆个为读⼊图⽚，可选的第三个参数，它针对
    # 特定的格式：对于JPEG，其表⽰的是图像的质量，⽤0 - 100的整数表⽰，默认95;对于png ,第三个参数表⽰的是
    # 压缩级别， 默认为3。
    cv2.imwrite("/home/ucar/pic/" + str(num) + ".jpg", frame)
    # 释放视频流
    cap.release()
    # 关闭所有窗口
    cv2.destroyAllWindows()

# 设置目标点
def set_target(move_base, target):
    # 从move_base_msgs.msg中接受目标信息
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    # 头时间戳设置为现在的时间
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target))
    move_base.send_goal(goal)

# 记忆赛道的速度发布
def go_pub(pub, v, t):
    twist = Twist()
    twist.linear.x = v
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0
    time = 0
    end = t*t
    while(time < end):
       pub.publish(twist)
       time += 1

# 发布速度
def twist_pub(pub, lx, ly, lz, ax, ay, az):
    twist=Twist()
    # 三个方向的线速度
    twist.linear.x=lx
    twist.linear.y=ly
    twist.linear.z=lz
    # 三个方向的角速度
    twist.angular.x=ax
    twist.angular.y=ay
    twist.angular.z=az
    pub.publish(twist)

# # 检测二维码
# def cvDICT():
#     cap=cv2.VideoCapture("/dev/ucar_video")
#     ids=None
#     i=0
#     while ids == None:
#         i += 1
#         rospy.loginfo(f"while in {i}")
#         ret, frame=cap.read()
#         cv2.imwrite(f"/home/ucar/outputs/te2t{int(i/10)}.jpg", frame)
#         frame=cv2.flip(frame, 1)  # 图像左右颠倒
#         gray=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         aruco_dict=cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
#         parameters=cv2.aruco.DetectorParameters_create()
#         # dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
#         # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
#         corners, ids, rejectedImgPoints=cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
#         time.sleep(0.1)
#     rospy.loginfo("while quit")
#     cap.release()
#     cv2.destroyAllWindows()
#     return ids

def d2_test():
    # 使用global声明wake_key的全局变量类型，就可以在函数中更改
    global wake_key 
    # 图片的序号初始化
    num = 1
    # 这三个数是官方例程里给出的，暂时不清楚作用
    dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    # 使用默认字体
    font = cv2.FONT_HERSHEY_SIMPLEX
    # 设置目标点
    target_B = Pose(Point(4.487, -4.949, 0.000),
                  Quaternion(0.000, 0.000, -0.266, 0.964))
    target_C = Pose(Point(3.124, -4.980, 0.000),
                  Quaternion(0.000, 0.000, -0.366, 0.931)
    target_D = Pose(Point(1.895, -3.980, 0.000),
                  Quaternion(0.000, 0.000, 0.900, -0.335))
    target_E = Pose(Point(0.130, -1.100, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.009))
    # 创建 SimpleActionServer类，
    # 'move_base'为动作服务器的名称，这个名称会成为其所有子话题的命名空间，
    #  MoveBaseAction为动作服务器的类型
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    # 等待连接服务器，5s等待时间限制
    # actionlib.SimpleActionClient.wait_for_server(rospy.Duration())的返回值为True or False
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server")
    # 判断条件为麦克风阵列被唤醒
    if wake_key != 'Jesus':            # a != '370'
        # 设置开始时间戳
        start_time = rospy.Time.now()                     
        # 例：pub1 = rospy.Publisher(“/topic_name”, message_type, queue_size=size)
        # /topic_name表示发布者向这个topic发布消息。message_type表示发布到话题中消息的类型。
        # 若queue_size是 None 则为同步通讯方式，若为整数则为异步通讯方式。
        #  pub为向cmd_vel节点异步发送Twist消息
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # pub1为向bobao节点异步发送Int32消息
        pub1 = rospy.Publisher('bobao', Int32, queue_size=1)
        # pub2 = rospy.Publisher('restart',Int32,queue_size=1)
        # pub3为向yolo_wake节点异步发送Int32消息
        pub3 = rospy.Publisher('yolo_wake', Int32, queue_size=4)
        rospy.loginfo("OK1")
        # 向B区进发
        # target = target_B  # 2区目标点 1.400  ;1.59
        # set_target(move_base, target)
	    # # rospy.loginfo("Head for targetB!"
        # while True:
        #     state = move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         break
        # go_pub(pub, 0.5, 110)
        # take_photo(num)  # photo1
        # while True:
        #     result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
        #     orz = result.pose.pose.orientation.z
        #     if orz > -0.95 and orz < -0.8:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        #         break
        #     elif orz < -0.95:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9)
        #     elif orz > -0.8:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9)
        #     else:
        # num += 1
        # take_photo(num)  # photo2
        # # 向C区进发
        # target=target_C  # C区目标点 3.14
        # set_target(move_base, target)
	    # # rospy.loginfo("Head for targetC!")
        # while True:
        #     state=move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        # # client.update_configuration({"max_vel_x": 0.2})
        # # client.update_configuration({"max_vel_y": 0.3})
        #         num += 1
        #         take_photo(num)  # photo3
        #         break
        # # 向D区进发
        # target=target_D  # D区目标点
        # set_target(move_base, target)
	    # # rospy.loginfo("Head for targetD!")
        # while True:
        #     state=move_base.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         break
        # go_pub(pub, 0.6, 165)
        # num += 1
        # take_photo(num)  # photo4
        # while True:
        #     result=rospy.wait_for_message( '/amcl_pose', PoseWithCovarianceStamped, timeout=None)
        #     orz=result.pose.pose.orientation.z
        #     if orz > -0.55 and orz < -0.35:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        #         break
        #     elif orz < -0.55:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8)
        #     elif orz > -0.35:
        #         twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8)
        #     else:
        # go_pub(pub, 0.70, 130)
        # num += 1
        # take_photo(num)  # photo5
        # # 向E区（终点）进发
        goto_target(num, target_B)
        num += 2
        goto_target(num, target_C)
        num += 1
        goto_target(num, target_D)
        num += 2
        set_target(move_base, target_E)
        # 向终点进发时判断是否到达需要加上时间判据
        rospy.loginfo("dash")
        # 设置五分钟时间限制
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))
        # 计算运行所用时间
        running_time = rospy.Time.now() - start_time
         # 把运行所用时间转化为秒数
        running_time=running_time.secs
        # 查看是否成功到达
        rospy.loginfo("check")
        while True:
            state = move_base.get_state()
            # 到达
            if state == GoalStatus.SUCCEEDED:
                break
            # 超时
            if not finished_within_time:
                break
        if state == GoalStatus.SUCCEEDED:
            pub3.publish(wake_key)
            pub1.publish(wake_key)
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        else:
            rospy.loginfo("Failed!")

def listener():
    # 使用global声明wake_key的全局变量类型，就可以在函数中更改
    global wake_key
    # 节点初始化，定义一个匿名节点，可以同时运行多个数量的节点
    rospy.init_node('move_test', anonymous=True)
    # 等待麦克风的唤醒角度, 语音输入
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout=None)
    d2_test()

# 关闭各节点
def kill_all_nodes():
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

# 向目标点进发
def goto_target(num, target):
    set_target(move_base, target)
    while True:
        state = move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            break
    if num == 1:
        go_pub(pub, 0.5, 110)
        take_photo(num)
        while True:
            result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
            orz = result.pose.pose.orientation.z
            if orz > -0.95 and orz < -0.8:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                break
            elif orz < -0.95 and orz == -0.95:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9)
            elif orz > -0.8 and orz == -0.8:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 
                0.0, -0.9)
            else:
        take_photo(num + 1)
    elif num == 3:
        take_photo(num)
    elif num == 4:
        go_pub(pub, 0.6, 165)
        take_photo(num)
        while True:
            result = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=None)
            orz = result.pose.pose.orientation.z
            if orz > -0.55 and orz < -0.35:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                break
            elif orz < -0.55 and orz == -0.55:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8)
            elif orz > -0.35 and orz == -0.35:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8)
            else:
        go_pub(pub, 0.70, 130)
        take_photo(num + 1)
    else:
      


if __name__ == "__main__":
    listener()

#####################################################
