#!/usr/bin/env python
# -*- coding: utf-8 -*-

#####################################################

# Author:周毅
# Date:2022.7.30

#####################################################

import sys
import rospy
import roslib
import dynamic_reconfigure.client
import math
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32

# 定义全局变量
flag_1 = 0
flag_2 = 0

# 订阅reach_target节点，用于判断主函数是否发送更新参数的标志
# def Flag_Sub():
#     rospy.Subscriber('reach_target', Int32, TargetD_Callback)
#     while not rospy.is_shutdown():
#         rospy.spin()

# def TargetD_Callback(data):
#     global flag
#     rospy.loginfo("Subscribe Successfully!")
#     flag = 1

def Callback(config):
    rospy.loginfo("Request Successfully!")

if __name__ == "__main__":
    # global flag_1
    # global flag_2
    rospy.init_node("dynamic_client")
    client = dynamic_reconfigure.client.Client("/move_base/TebLocalPlannerROS", timeout = 30, config_callback = Callback)
    r = rospy.Rate(50)
    rospy.loginfo("Reading position...")
    # Flag_Sub()
    passed_words = 0
    while not rospy.is_shutdown():
        var = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout = None)
        x = var.pose.pose.position.x
        y = var.pose.pose.position.y
        # z = var.pose.pose.orientation.z
        # w = var.pose.pose.orientation.w     
        # Having passed the korridor   
        if x > 4.5 and x < 5.0 and y > -0.5 and y < -0.0 and flag_1 == 0:
            #client.update_configuration({"max_vel_x": 1.0})
            #client.update_configuration({"max_vel_y": 1.0})
            #client.update_configuration({"max_vel_theta": 6.3})
            #client.update_configuration({"acc_lim_x": 1.0})
            #client.update_configuration({"acc_lim_y": 1.0})
            #client.update_configuration({"acc_lim_theta": 3.5})
            client.update_configuration({"feasibility_check_no_poses": 2})
            client.update_configuration({"global_plan_viapoint_sep": 1.0})
            client.update_configuration({"max_global_plan_lookahead_dist": 1.2})
            #client.update_configuration({"min_obstacle_dist": 0.055})
            client.update_configuration({"weight_kinematics_turning_radius": 15.0})#20
            client.update_configuration({"weight_kinematics_nh": 25.0})#25
            #client.update_configuration({"penalty_epsilon": 0.1})
            client.update_configuration({"weight_viapoint": 10.0})
            client.update_configuration({"weight_optimaltime": 4.0})
            client.update_configuration({"weight_shortest_path": 0.3})
            #client.update_configuration({"weight_inflation": 150.0})
            client.update_configuration({"weight_obstacle": 600.0})
            client.update_configuration({"weight_kinematics_forward_drive": 400.0})
            rospy.loginfo("After the korridor, changed!")
            flag_1 = 1
        # Heading for the terminal
        elif x > 0.0 and x < 2.3 and y > -6.0 and y < -2.5 and passed_words == Int32(1) and flag_2 == 0:
            client.update_configuration({"yaw_goal_tolerance": 0.2})
            client.update_configuration({"xy_goal_tolerance": 0.08})
            rospy.loginfo("Heading for the terminal, changed!")
            flag_2 == 1
        else:
            pass
        if flag_1 == 1:
            passed_words = rospy.wait_for_message('reach_target', Int32, timeout = None)
            rospy.loginfo(passed_words)
            rospy.loginfo(type(passed_words))
        r.sleep()

