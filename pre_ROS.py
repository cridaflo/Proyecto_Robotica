#!/usr/bin/env
import cv2
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np
import rospy
import roslib
import sys, time

im =  cv2.imread("photo_1.jpg")
clf = joblib.load("clasificador.pkl")
rta = ''
empezar = False

def fotoCallBack(msg):#CallBack del topico que toma la foto
    global empezar
    empezar = msg.data

def identify():#identifica la foto de la camara
    global im
    global clf

#carga el clasificador
    clf = joblib.load("clasificador.pkl")
    im =  cv2.imread("photo_1.jpg")
    empezar = False
# Convert to Grayscale and apply Gaussian filtering
im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)#pasa la imagen a escala de grises
im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)#aplica filtro Gauss

# Threshold the image
#el valor de 90 se calibra

#metodo threshold
ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)

# Find contours in the image
cont, ctrs, hier = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#contornos de la imagen

# Get rectangles contains each contour
rects = [cv2.boundingRect(ctr) for ctr in ctrs]#rectangles de cada numero

# For each rectangular region, calculate HOG features and predict
# the digit using Linear SVM.
nums = []
for rect in rects:
    # Draw the rectangles
    cv2.rectangle(im, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3)
    # Make the rectangular region around the digit
    leng = int(rect[3] * 1.6)
    pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
    pt2 = int(rect[0] + rect[2] // 2 - leng // 2)
    roi = im_th[pt1:pt1+leng, pt2:pt2+leng]
    # Resize the image
    roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
    roi = cv2.dilate(roi, (3, 3))
    # Calculate the HOG features
    roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
    nbr = clf.predict(np.array([roi_hog_fd], 'float64'))
    cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)
    nums.append((rect[0], nbr[0]))  #
    nums.sort()  # organiza el array
    rta =''

    for x in nums:
            rta+=str(x[1])
    print rta

    cv2.imshow("Resulting Image with Rectangular ROIs", im)#Muestra resultado


def finale():#ROS- link
    global empezar
    global rta
    rospy.init_node('tomaFoto', anonymous=False)#crea nodo RECON
    rospy.Subscriber('empezar_adivinar', Boolean, fotoCallBack )#se suscribe a empezar_adivinar
    rospy.Publisher('password_guess', String, queue__size = 10)
    rate = rospy.Rate(10)# 10 H

    while not rospy.is_shutdown():
        empzar = True
        if empezar:
            tomarFoto()
            rta = identify()
            pub.publish(rta)
            rate.sleep()

if __name__ == '__main__':
    try:
        finale()
    except rospy.ROSInterruptException:
        pass
