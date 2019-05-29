#!/usr/bin/env
import sys, time
import numpy as np
import cv2
from master_msgs_iele3338.msg import Boolean
import roslib
import rospy
import cv2

camera = cv2.VideoCapture(0)
empezar = False
def tomarFoto():
    globla camera
    return_value, image = camera.read()
    cv2.imwrite('opencv'+str(1)+'.png', image)
    del(camera)

def adivinar(msg):
    global empezar
    empezar = msg.data

def finale():
    global empezar
    rospy.init_node('tomaFoto', anonymous=True)
    rospy.Subscriber('adivinar', Boolean, adivinar)
    while not rospy.is_shutdown():
        if empezar:
            tomarFoto()

if __name__ == '__main__':
    try:
        iniciar()
    except rospy.ROSInterruptException:
        pass


