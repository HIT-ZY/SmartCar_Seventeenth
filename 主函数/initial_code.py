#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
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


def take_photo(pub, num, wake_key):
    cap = cv2.VideoCapture("/dev/ucar_video")
    
    rospy.loginfo("take photo-%s",str(num))
    weight = 640
    height = 480
    cap.set(3,weight)  # 设置分辨率 3和4 分别代表摄像头的属性值。你可以使用函数 cap.get(propId) 来获得视频的一些参数信息。这里propId 可以是 0 到 18 之间的任何整数。每一个数代表视频的一个属性,见表其中的一些值可以使用cap.set(propId,value) 来修改,value 就是你想要设置成的新值。例如,我可以使用 cap.get(3) 和 cap.get(4) 来查看每一帧的宽和高。默认情况下得到的值是 640X480。但是我可以使用 ret=cap.set(3,320)和 ret=cap.set(4,240) 来把宽和高改成 320X240。
    cap.set(4,height)
    codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
    print(codec)
    cap.set(cv2.CAP_PROP_FOURCC, codec)
    ret, frame = cap.read()
    if num==3:
        frame = frame[30:360,40:480].copy()
    cv2.waitKey(20)
    cv2.imwrite("/home/ucar/pic/" + str(num) + ".jpg", frame)
    
    cap.release()
    cv2.destroyAllWindows()


def set_target(move_base, target):
    goal = MoveBaseGoal()
    goal.target_pose.pose = target
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    rospy.loginfo("Going to: " + str(target))
    move_base.send_goal(goal)

def go_pub(pub,v,t):
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

def twist_pub(pub,lx,ly,lz,ax,ay,az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    pub.publish(twist)  


def cvDICT():
    cap = cv2.VideoCapture("/dev/ucar_video")
    
    ids = None
    i = 0
    while ids == None:
        i += 1
        rospy.loginfo(f"while in {i}")
        ret, frame = cap.read()
        cv2.imwrite(f"/home/ucar/outputs/te2t{int(i/10)}.jpg", frame)
        frame = cv2.flip(frame, 1)  # 图像左右颠倒
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        # dst1 = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        time.sleep(0.1)
        
    rospy.loginfo("while quit")
    cap.release()
    cv2.destroyAllWindows()
    return ids

#####################################################


wake_key = 'Nobody is here but us chickens.' #a = '370'


def d2_test():
    global wake_key
    num = 0
    flag = 0
    flag1 = 0

    dist = np.array(([[-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275]]))
    newcameramtx = np.array([[189.076828, 0., 361.20126638]
		                    , [0, 2.01627296e+04, 4.52759577e+02]
		                    , [0, 0, 1]])
    mtx = np.array([[398.12724231, 0., 304.35638757],
		        [0., 345.38259888, 282.49861858],
		        [0., 0., 1.]])

    font = cv2.FONT_HERSHEY_SIMPLEX
    loca = False
    #target_B = Pose(Point(5.137,-5.013,0.000), Quaternion(0.000,0.000,-0.197,0.980))
    target_B = Pose(Point(4.487, -4.949, 0.000),Quaternion(0.000, 0.000, -0.266, 0.964))
#Position(4.487, -4.949, 0.000), Orientation(0.000, 0.000, -0.266, 0.964)
    target_C = Pose(Point(3.124,-4.980,0.000), Quaternion(0.000,0.000,-0.366,0.931))
    #target_D = Pose(Point(1.076,-5.083,0.000), Quaternion(0.000,0.000,-0.255,0.967))
    #target_D = Pose(Point(1.882, -4.580, 0.000), Quaternion(0.000, 0.000, 0.962, -0.272))
    target_D = Pose(Point(1.895, -3.980, 0.000), Quaternion(0.000, 0.000, 0.900, -0.335))
#Position(2.002, -4.580, 0.000), Orientation(0.000, 0.000, 0.962, -0.272)
    target_E = Pose(Point(0.130,-1.100,0.000), Quaternion(0.000,0.000,1.000,0.009))
    
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    # 等待连接服务器，5s等待时间限制
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server")

    if wake_key != 'Jesus': # a != '370'
        start_time = rospy.Time.now()
        
        pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        pub1=rospy.Publisher('bobao',Int32,queue_size=1)
        #pub2=rospy.Publisher('restart',Int32,queue_size=1)
        pub3=rospy.Publisher('yolo_wake',Int32,queue_size=4)

        #num += 1
        #take_photo(pub3, num, wake_key) # photo1
        rospy.loginfo("OK1")

        target = target_B  # 2区目标点 1.400  ;1.59
        set_target(move_base, target)
	#rospy.loginfo("Head for targetB!")

        while True:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
        num += 1
        go_pub(pub,0.5,110)
        take_photo(pub3, num, wake_key) # photo4
        while True:
            result = rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped,timeout=None)
            orz = result.pose.pose.orientation.z
            if orz > -0.95 and orz < -0.8:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                break
            elif orz < -0.95:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, 0.9)
            elif orz > -0.8:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, -0.9)
            elif orz < -1.1:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, 1.3)
        num += 1
        take_photo(pub3, num, wake_key) # photo5
        
        target = target_C  # C区目标点 3.14
        set_target(move_base, target)
	#rospy.loginfo("Head for targetC!")

        while True:     
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
        #       client.update_configuration({"max_vel_x": 0.2})
        #       client.update_configuration({"max_vel_y": 0.3})
                num += 1
                take_photo(pub3, num, wake_key) # photo3
                break

        target = target_D  # D区目标点
        set_target(move_base, target)
	#rospy.loginfo("Head for targetD!")
        
        while True:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
        go_pub(pub,0.6,165)
        num += 1
        take_photo(pub3, num, wake_key) # photo4

        while True:
            result = rospy.wait_for_message('/amcl_pose',PoseWithCovarianceStamped,timeout=None)
            orz = result.pose.pose.orientation.z
            if orz > -0.55 and orz < -0.35:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                break
            elif orz < -0.55:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, 0.8)
            elif orz > -0.35:
                twist_pub(pub,0.0, 0.0, 0.0, 0.0, 0.0, -0.8)
        go_pub(pub,0.70,130)
        num += 1
        take_photo(pub3, num, wake_key) # photo5

        target = target_E # D1终点
        set_target(move_base, target)
        
        rospy.loginfo("dash")
        # 五分钟时间限制
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))
        # 运行所用时间
        running_time = rospy.Time.now() - start_time
        running_time = running_time.secs
        # 查看是否成功到达
        rospy.loginfo("check")

        while True:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
    
                break
            if not finished_within_time:
                break
        if state == GoalStatus.SUCCEEDED:
            pub3.publish(wake_key)
            pub1.publish(wake_key)
            rospy.loginfo("Goal succeeded! Time Result: %f", running_time)
        else:
            rospy.loginfo("Failed!")
        
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


def listener():
    global wake_key
    # 节点初始化
    rospy.init_node('move_test', anonymous=True)
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout=None)
    d2_test()


if __name__ == "__main__":
    listener()


            
#####################################################
