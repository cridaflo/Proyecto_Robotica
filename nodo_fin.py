#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import *
from std_msgs.msg import Int32


# Obstacle[] obstaclesc
password = -1

def endService():
    global password
    req_estate = 0
    while req_estate == 0:
        if password >=0:
            end_service = rospy.ServiceProxy('end_service', EndService)
            end = end_service(password)
            req_estate = end.correct
            print(req_estate)
    return req_estate

def passwordCallback(msg):
    global password
    password = msg.data

def finale():
    rospy.init_node('master_end', anonymous=True)
    rospy.Subscriber('password_guess', Int32, passwordCallback)
    end = endService()
    rospy.spin()




if __name__ == '__main__':


    try:
        finale()
    except rospy.ROSInterruptException:
        pass

