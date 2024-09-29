#!/usr/bin/env python
# coding:utf-8

# 加载所需模块
import rosnode
import numpy
import rospy
import time
import os
import sys
import cv2
from std_msgs.msg import String
from std_msgs.msg import Int32
from pathlib import Path
from photo_and_qr.srv import *
from geometry_msgs.msg import Twist

def Twist_Pub(pub, lx, ly, lz, ax, ay, az):
    twist = Twist()
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz
    twist.angular.x = ax
    twist.angular.y = ay
    twist.angular.z = az
    pub.publish(twist)

def Server_Srv():
    rospy.init_node("PhotoAndQR_Server")
    s = rospy.Service("greetings", Greeting, Handle_Function)
    rospy.loginfo("Ready to handle the request:")
    # 阻塞至程序结束
    rospy.spin()

def Handle_Function(req):
    cvDICT()
    return GreetingResponse("Finished!")

def cvDICT():
    dist_matrix = np.array([-0.323074221732616, 0.103243450673514, 0, 0, 0])
    camera_matrix = np.array([[431.416676013977, 0., 321.918201475723], [0., 433.136389691851, 229.497858115603], [0., 0., 1.]]
    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    cap = cv2.VideoCapture("/dev/ucar_video")
    while True:
        ret, frame = cap.read()
        cv2.waitKey(20)
        # 图像左右颠倒
        frame = cv2.flip(frame, 1)  
        width, height = 480, 600
        # 读取摄像头画面
        # 纠正畸变
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (height, width), 0, (height, width))
        dst = cv2.undistort(frame, camera_matrix, dist_matrix, None, newcameramtx)
        x, y, width, height = roi
        dst = dst[y:y + height, x:x + width]
        frame = dst
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
        (corners, ids, rejectedImgPoints) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        if ids is not None:
            break
    for markerID, markerCorner in zip(ids,corners):
        if markerID == [0]:
            # rvec为旋转矩阵，tvec为位移矩阵
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, 0.05, camera_matrix, dist_matrix)
            # get rid of that nasty numpy value array error
            (rvec - tvec).any()  
            # 距离估计
            distance = tvec[0][0][2] * 178.2 + 0.6
            if distance >= 18:
                start_time = rospy.Time.now().to_sec()
                running_time = rospy.Time.now().to_sec() - start_time
                set_time = (distance - 15) / 100 / 0.5
                while running_time < set_time:
                    running_time = rospy.Time.now().to_sec() - start_time
                    Twist_Pub(pub_cmd, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
                rospy.loginfo("Have headed!Arrived!")
            else:
                rospy.loginfo("Not need to head!Arrived!")

if __name__ == "__main__":
    Server_Srv()
