#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import *
from std_msgs.msg import Int32, Bool


# Obstacle[] obstaclesc
adivinar = False
password = 1234


def passwordCallback(msg):
    global adivinar
    adivinar = msg.data

def adivinar():
    global adivinar, password
    rospy.init_node('adivinador', anonymous=True)
    rospy.Subscriber('empezar_adivinar', Bool, passwordCallback)
    pub = rospy.Publisher('password_guess', Int32, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if adivinar:
            pub.publish(password)
        rate.sleep()


    rospy.spin()




if __name__ == '__main__':


    try:
        adivinar()
    except rospy.ROSInterruptException:
        pass

