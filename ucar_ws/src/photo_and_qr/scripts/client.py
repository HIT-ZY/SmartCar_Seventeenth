#!/usr/bin/env python
# coding:utf-8

#####################################################

# Author:周毅
# Date:2022.7.11

#####################################################

# 加载所需模块
import rospy
from PhotoAndQR.srv import *

def Client_Srv():
    rospy.init_node('PhotoAndQR_Client')
    rospy.wait_for_service("greetings")
    try:
        greetings_client = rospy.ServiceProxy("greetings", Greeting)
        response = PhotoAndQR_Client.call(position, num, flag)
        rospy.loginfo("Message From server:%s"%response.feedback)
    except rospy.ServiceException, e:
        rospy.logwarn("Service call failed: %s"%e)

if __name__=="__main__":
    Client_Srv()
