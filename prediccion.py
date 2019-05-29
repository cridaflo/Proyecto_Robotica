# Import the modules
# #!/usr/bin/env
import cv2
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np
import rospy
import roslib
import sys, time
#import scipy.ndimage import filters
from sensor_msgs.msg import CompressedImage

# carga el clasificador
clf = joblib.load("clasificador.pkl")

# carga la imagen de prueba
def __init__(self): #inicia el nodo del que se suscribe para recibir la imagen

            self.image_pub = rospy.Publisher("envio_Compressed", CompressedImage)#topico envio_Compressed donde publica la imagen comprimida de salida
            self.subscriber = rospy.Subscriber("envio_Imagen", CompressedImage, self.callback, queue_size = 1)#topico que toma la foto
pass


im = cv2.imread("photo_4.jpg")

# Convert to Grayscale and apply Gaussian filtering
im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)#pasa la imagen a escala de grises
im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)#aplica filtro Gauss

# Threshold the image
#el valor de 90 se calibra

#metodo threshold(imagen, umbral, val_max
ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)

# Find contours in the image
cont, ctrs, hier = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#contornos de la imagen

# Get rectangles contains each contour
rects = [cv2.boundingRect(ctr) for ctr in ctrs]#rectangles de cada numero

# For each rectangular region, calculate HOG features and predict
# the digit using Linear SVM.
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

resultado = cv2.imshow("Resulting Image with Rectangular ROIs", im)
cv2.waitKey()


#crea el mensaje que manda a envioImagen CompressedImage
msg = CompressedImage
msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
self.image_pub.publish(msg) #publica en el topico el resultado

def nodo_Foto():
    rospy.init_node('tomar_Foto', anonymous=True)
    pub = rospy.Publisher('Recibe_foto', String, queue_size=10)
    rate = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():
        pub.publish('formato_foto')
        rate.sleep()
pass


if __name__ == '__main__':#ejecuta el main
    try:
        #nodo_Foto()
    except rospy.ROSInterruptException:
        pass

