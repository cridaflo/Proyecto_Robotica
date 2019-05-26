#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import *
from std_msgs.msg import Int32


# Obstacle[] obstaclesc
password = -1
estado  = 0
req_estate = 0

def endService():
    global password, req_estate
    print('estoy adivinando', password)
    if password >=0:
        end_service = rospy.ServiceProxy('end_service', EndService)
        end = end_service(password)
        req_estate = end.correct


def passwordCallback(msg):
    global password
    password = msg.data

def estadoCallback(msg):
    global estado
    estado = msg.data

def finale():
    global req_estate
    rospy.init_node('master_end', anonymous=True)
    rospy.Subscriber('password_guess', Int32, passwordCallback)
    rospy.Subscriber('el_estado', Int32, estadoCallback)
    pub6 = rospy.Publisher('el_estado',Int32, queue_size = 2)
    while not rospy.is_shutdown():
        if estado == 3:
            endService()
            if req_estate!=0:
                pub6.publish(4)
                break
     
    rospy.spin()




if __name__ == '__main__':


    try:
        finale()
    except rospy.ROSInterruptException:
        pass

