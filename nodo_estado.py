#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle
from master_msgs_iele3338.srv import AckService, StartService
from std_msgs.msg import Float64, Int32
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import numpy as np
import threading
import time
import os
import matplotlib.pyplot as plt
import math
import Queue as queue

estado = 0

def estadoCallback(msg):
    global estado    
    estado = msg.data

def estado0():
    print(0)
    pass

def estado1():
    print(1)
    pass

def estado2():
    print(2)
    pass

def estado3():
    print(3)
    pass

def estado4():
    print(4)
    pass
#funcion principal para controlar el pacman
def manejadorEstados():

    global estado

    #Se inicia el nodo, y se conecta con el topico pacmanActions0 donde va a publicar
    rospy.init_node('estado', anonymous=True)
    rospy.Subscriber('el_estado', Int32, estadoCallback)

    try:
       

        rate = rospy.Rate(10)  # 10hz
        
        i = 0
        while not rospy.is_shutdown():

            if estado == 0:
                estado0()
            elif estado ==1:
                estado1()
            elif  estado == 2:
                estado2()
            elif estado == 3:
                estado3()
            elif estado == 4:
                estado4()

            rate.sleep()

    except rospy.ServiceException as e:
        print("Error!!")


if __name__ == '__main__':
    try:
        manejadorEstados()
    except rospy.ROSInterruptException:
        pass
