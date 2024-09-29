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

wake_key='Nobody is here but us chickens.'       # a = '370'

#####################################################

#####################################################

def take_photo(num):
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("take photo-%s", str(num))
    width = 640
    height = 480
    cap.set(3,  width)
    cap.set(4,  height)
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    print(codec)
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    ret, frame=cap.read()
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

def twist_pub(pub, lx, ly, lz, ax, ay, az):
    twist=Twist()
    twist.linear.x=lx
    twist.linear.y=ly
    twist.linear.z=lz
    twist.angular.x=ax
    twist.angular.y=ay
    twist.angular.z=az
    pub.publish(twist)

def d2_test():
    global wake_key 
    num = 1
    dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    target_B = Pose(Point(4.487, -4.949, 0.000),
                  Quaternion(0.000, 0.000, -0.266, 0.964))
    target_C = Pose(Point(3.124, -4.980, 0.000),
                  Quaternion(0.000, 0.000, -0.366, 0.931))
    target_D = Pose(Point(1.895, -3.980, 0.000),
                  Quaternion(0.000, 0.000, 0.900, -0.335))
    target_E = Pose(Point(0.130, -1.100, 0.000),
                  Quaternion(0.000, 0.000, 1.000, 0.009))
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
        rospy.loginfo("Connected to move_base server")
    if wake_key != 'Jesus':            # a != '370'
        start_time = rospy.Time.now()                     
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub1 = rospy.Publisher('bobao', Int32, queue_size=1)
        pub3 = rospy.Publisher('yolo_wake', Int32, queue_size=4)
        rospy.loginfo("OK1")
        goto_target(num, target_B, move_base, pub)
        num += 2
        goto_target(num, target_C, move_base, pub)
        num += 1
        goto_target(num, target_D, move_base, pub)
        num += 2
        set_target(move_base, target_E)
        rospy.loginfo("dash")
        finished_within_time = move_base.wait_for_result(rospy.Duration(300))
        running_time = rospy.Time.now() - start_time
        running_time=running_time.secs
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
        kill_all_nodes()

def listener():
    global wake_key
    rospy.init_node('move_test', anonymous=True)
    wake_key = rospy.wait_for_message("/mic/awake/angle", Int32, timeout=None)
    d2_test()

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

def goto_target(num, target, move_base, pub):
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
            elif orz <= -0.95:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9)
            elif orz >= -0.8:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.9)
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
            elif orz <= -0.55:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8)
            elif orz >= -0.35:
                twist_pub(pub, 0.0, 0.0, 0.0, 0.0, 0.0, -0.8)
        go_pub(pub, 0.70, 130)
        take_photo(num + 1)
      
if __name__ == "__main__":
    listener()

#####################################################
