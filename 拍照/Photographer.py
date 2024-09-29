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

def take_photo(num):
    cap = cv2.VideoCapture("/dev/ucar_video")
    print("take photo-%s", str(num))
    width = 640
    height = 480
    cap.set(3,  width)
    cap.set(4,  height)
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    ret, frame=cap.read()
    cv2.waitKey(500)
    cv2.imwrite("/home/ucar/pic/" + str(num) + ".jpg", frame)
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    dist = np.array([-0.58650416, 0.59103816, -0.00443272, 0.00357844, -0.27203275])
    newcameramtx = np.array([[189.076828, 0., 361.20126638], [0, 2.01627296e+04, 4.52759577e+02], [0, 0, 1]])
    mtx = np.array([[398.12724231, 0., 304.35638757], [0., 345.38259888, 282.49861858],[0., 0., 1.]])
    font = cv2.FONT_HERSHEY_SIMPLEX
    while True:
        print("请输入开始的图片数：")
        num_begin = int(input())
        print("请输入结束的图片数：")
        num_end = int(input())
        while num_begin <= num_end:
            take_photo(num_begin)
            num_begin += 1
        print("请问要继续拍照吗，按任意健可继续，退出请按q:")
        corq_str = input()
        if corq_str == 'q':
            break
        else:
            pass
    print("这是第几张图-%s", str(num_begin))

#####################################################
