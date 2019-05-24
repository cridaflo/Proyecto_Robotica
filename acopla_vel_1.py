#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import AckService, StartService
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np
import threading
import time
import os
import matplotlib.pyplot as plt
import math
import Queue as queue

vel_izq = 0
vel_der = 0

def velIzqCallback(msg):
    global vel_izq
    vel_izq = msg.data

def velDerCallback(msg):
    global vel_der
    vel_der = msg.data

#funcion principal para controlar el pacman
def acoplador():

    global vel_izq, vel_der

    #Se inicia el nodo, y se conecta con el topico pacmanActions0 donde va a publicar
    rospy.init_node('acoplador', anonymous=True)
    pub = rospy.Publisher('motor_vel', Float32MultiArray, queue_size = 1)
    rospy.Subscriber('vel_izq', Float32, velIzqCallback)
    rospy.Subscriber('vel_der', Float32, velDerCallback)

    try:
       

        #se instancia el mensaje que sera publicado
        msg = Float32MultiArray()

        rate = rospy.Rate(20)  # 10hz
        

        while not rospy.is_shutdown():
            #se publica la direccion almacenda
            msg.data = [vel_izq, vel_der]
            pub.publish(msg)
            rate.sleep()

    except rospy.ServiceException as e:
        print("Error!! Make sure pacman_world node is running ")


if __name__ == '__main__':
    try:
        acoplador()
    except rospy.ROSInterruptException:
        pass
