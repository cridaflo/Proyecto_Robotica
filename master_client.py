#!/usr/bin/env python

import rospy
import sys
import threading
from master_msgs_iele3338.msg import Obstacle, Covariance
from master_msgs_iele3338.srv import AckService, StartService,StartServiceResponse
from std_msgs.msg import Float64, Bool
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist, Pose
import numpy as np
import threading
import time
import os
import matplotlib.pyplot as plt
import math
import Queue as queue


n_obs =0

vel_deseada = Float32MultiArray()

rate = 10
dt = 1/float(rate)
angulo = 0
x = 0
y = 0
xs= []
ys= []

kr = 1
kl = 1

cov = np.array([[1,0,0], [0,1,0], [0,0,1]])
error = []
tiempos = []
destino = [10, 10, np.radians(90)]
inicio = [0,0,0]

Empezo = False
sefini = False
calculo = False

tiempo = 0
eps = 50
pista_minx = 0
pista_maxx = 2500
pista_miny = 0
pista_maxy = 2500

saturacion = 3
saturacion2 = 1
minvel = 1.5

divisiones = 100.0
anchox = (pista_maxx-pista_minx)/divisiones
anchoy = (pista_maxy-pista_miny)/divisiones

puntosx =[]
puntosy = []

mundo = None


#parametros del controlador
kp = 3*0.09
ka = 8*0.15
kb = -1.5*0.14
rho = 0
alpha = 0
beta = 0


#variables relacionadas con la cinematica del robot
alfa1 = np.radians(90)
alfa2 = np.radians(-90)
beta1 = 0
beta2 = np.radians(180)
l = (0.26-0.0137)*1000
r = 0.0685*1000
x_robot = 0.52*1000
y_robot = 0.415*1000
diametro = np.sqrt(x_robot**2+y_robot**2)
#dimensiones de los cilindros
radios_C = []
centros_x = []
centros_y = []

J1_alt = np.array([[np.sin(alfa1 + beta1), -np.cos(alfa1 + beta1), -l * np.cos(beta1)]
                      , [np.sin(alfa2 + beta2), -np.cos(alfa2 + beta2), -l * np.cos(beta2)]])
J2 = np.array([[r, 0], [0, r]])
J2i = np.linalg.inv(J2)
R = np.array([[np.cos(inicio[2]), np.sin(inicio[2]), 0], [-np.sin(inicio[2]), np.cos(inicio[2]), 0], [0, 0, 1]])


req_estate = 0
pinto = False
terminoPintar = False

def start_service(msg):
    global obstacles_x, obstacles_x, n_obs,inicio, destino, x, y, angulo, resp
    n_obs = msg.n_obstacles
    obstacles = msg.obstacles

    for i in range(0, n_obs):
        o = obstacles[i]
        op = o.position.position
        centros_x.append(op.x)
        centros_y.append(op.y)
        radios_C.append(o.radius)


    posXi = msg.start.position.x
    posYi = msg.start.position.y
    orienti = msg.start.orientation.w
    x = posXi
    y = posYi
    angulo = orienti
    inicio = [posXi, posYi, orienti]

    posXf = msg.goal.position.x
    posYf = msg.goal.position.y
    orientf = msg.goal.orientation.w

    destino = [posXf, posYf, orientf]
    print(destino)
    print('datos_cargados')
    pintar()
    A_estrella()
    resp = StartServiceResponse()
    #resp.response = []
    return resp


    


def ackService():
    global req_estate
    #rospy.init_node('master_client_py', anonymous=True)
    grupo = 14
    ipaddr = "157.253.210.30"
    ack_service = rospy.ServiceProxy("ack_service", AckService)

    while req_estate == 0 and not rospy.is_shutdown():
        
        #print('debug_1')
        ack = ack_service(group_number=grupo, ip_address=ipaddr)
        #print('debug_2')
        req_estate = ack.state
        #print(req_estate)

    print('preholi')

    return req_estate

def posCallback(msg):
    global x, y, xs, ys, xs_teo, ys_teo, angulo
    global J1, vel, J2, J1i, sefini
    global Empezo, tiempo, tiempos
    global alpha, rho, beta, R, error, dt, l, r, cov, kr, kl

    vels = msg.data
    
    velIzq = vels[0]
    velDer = vels[1]

    dsr =velDer*dt
    dsr= velIzq*dt

    ds = (dsr+dsl)/2.0
    dtheta = (dsr-dsl)/l

    a_int = angulo+dtheta/2.0
    dx = ds*np.cos(a_int)
    dy = ds*np.cos(a_int)
    
    Fp = np.array([[1,0,-dy], [0,1,dx], [0,0,1]])
    cos_int = np.cos(a_int)
    sin_int = np.sin(a_int)
    Fs11 = cos_int*0.5-ds*sin_int/(2*l)
    Fs12 = cos_int*0.5+ds*sin_int/(2*l)
    Fs21 = sin_int*0.5+ds*cos_int/(2*l)
    Fs22 = sin_int*0.5-ds*cos_int/(2*l)
    Fs = np.array([[Fs11, Fs12],[Fs21, Fs22], [1.0/l, -1.0/l]])

    M1 = np.dot(cov, np.transpose(Fp))
    M1 = np.dor(Fp, M1)

    Sigma = np.array([[kr*abs(dsr), 0], [0, kl*abs(dsl)]])
    M2 = np.dot(Sigma, np.transpose(Fs))
    M2 = np.dot(Fs, M2)

    cov = M1+M2
    x = x+dx
    y = y+dy
    angulo = angulo+dtheta

    pass


# Callback que lee la posicion del robot. Ademas dentro de esta misma funciona se implementan las ecuaciones de la cinematica
# del robot para calcular la posicion teoriica del robot
def posCallbackT(msg):
    global x, y, xs, ys, xs_teo, ys_teo, angulo
    global J1, vel, J2, J1i, sefini
    global Empezo, tiempo, tiempos
    global alpha, rho, beta, R, error

    linear = msg.linear
    angular = msg.angular
    x = linear.x
    y = linear.y
    angulo = float(angular.z)

    if sefini == False:
        if Empezo == False:

            xs_teo.append(x)
            ys_teo.append(y)
            xs.append(x)
            ys.append(y)
            error.append(np.sqrt((xs_teo[-1] - xs[-1]) * 2 + (ys_teo[-1] - ys[-1]) * 2))
            tiempos.append(tiempo)

        else:

            R = np.array([[np.cos(angulo), np.sin(angulo), 0], [-np.sin(angulo), np.cos(angulo), 0], [0, 0, 1]])
            Ri = np.linalg.inv(R)
            A = np.dot(Ri, J1i)
            v = np.array([vel.data[0], vel.data[1]])
            D = np.dot(J2, v)
            D = np.append(D, [0])
            si = np.dot(A, D)
            tact = tiempo
            dt = tact - tiempos[-1]
            dx = si[0] * dt
            dy = si[1] * dt
            xs_teo.append(xs_teo[-1] + dx)
            ys_teo.append(ys_teo[-1] + dy)
            xs.append(x)
            ys.append(y)
            error.append(error[-1] + np.sqrt((xs_teo[-1] - xs[-1]) ** 2 + (ys_teo[-1] - ys[-1]) ** 2))
            tiempos.append(tact)

    Empezo = True
    pass




# funion que crea la descomposicion en celdas del mapa
def pintar():
    global mundo, centros_x, centros_y, radios_C, divisiones, inicio
    global anchox, anchoy, pinto, terminoPintar, pista_minx, pista_miny
    global puntosx, puntosy, destino, destino_g, destino_p, raiz    

    div = int(divisiones)
    mundo = np.chararray((div, div))
    mundo.fill('.')
    for i in range(0, div):
        for j in range(0, div):
            x_prom = pista_minx + ((2 * i + 1) * anchox) / 2.0
            y_prom = pista_miny + ((2 * j + 1) * anchoy) / 2.0
            n = len(centros_x)
            #print('n: %s', n)
            for k in range(0, n):

                cx = centros_x[k]
                cy = centros_y[k]
                cr = radios_C[k]
                dx = (x_prom - cx) ** 2
                dy = (y_prom - cy) ** 2
                dist = np.sqrt(dx + dy)

                if dist <= cr:

                    mundo[i][j] = '#'
                    pxs = [pista_minx + i * anchox, pista_minx + i * anchox, pista_minx + (i + 1) * anchox,
                            pista_minx + (i + 1) * anchox]
                    pys = [pista_miny + j * anchoy, pista_miny + (j + 1) * anchoy,
                            pista_miny + (j + 1) * anchoy, pista_miny + j * anchoy]
                    puntosx.append(pxs)
                    puntosy.append(pys)

    destinox = int((destino[0] - pista_minx) / anchox)
    destinoy = int((destino[1] - pista_miny) / anchoy)
    print(destinox, destinoy)
    dx = [pista_minx + destinox * anchox, pista_minx + destinox * anchox, pista_minx + (destinox + 1) * anchox,
            pista_minx + (destinox + 1) * anchox]
    dy = [pista_miny + destinoy * anchoy, pista_miny + (destinoy + 1) * anchoy,
            pista_miny + (destinoy + 1) * anchoy, pista_miny + destinoy * anchoy]
    mundo[destinox][destinoy] = 'X'
     
    rx = int((inicio[0] - pista_minx) / anchox)
    ry = int((inicio[1] - pista_miny) / anchoy)
    raiz = (rx, ry)
    mundo[rx][ry] = 'R'
    destino_g = (destinox, destinoy)
    destino_p = (dx, dy)
    



# retorna una lita con los putnos adyacentes a las corrdenadas dadas
def puntos(punto):
    global divisiones, mundo
    x = punto[0]
    y = punto[1]
    rta = []
    if x > 0 and not mundo[x - 1][y] == '#':
        rta = rta + [(x - 1, y)]
    if x < divisiones - 1 and not mundo[x + 1][y] == '#':
        rta = rta + [(x + 1, y)]
    if y > 0 and not mundo[x][y - 1] == '#':
        rta = rta + [(x, y - 1)]
    if y < divisiones - 1 and not mundo[x][y + 1] == '#':
        rta = rta + [(x, y + 1)]
    if x > 0 and y > 0 and not mundo[x - 1][y - 1] == '#':
        rta = rta + [(x - 1, y - 1)]
    if x > 0 and y < divisiones - 1 and not mundo[x - 1][y + 1] == '#':
        rta = rta + [(x - 1, y + 1)]
    if x < divisiones - 1 and y > 0 and not mundo[x + 1][y - 1] == '#':
        rta = rta + [(x + 1, y - 1)]
    if x < divisiones - 1 and y < divisiones - 1 and not mundo[x + 1][y + 1] == '#':
        rta = rta + [(x + 1, y + 1)]
    return rta


def cartesianas(nodo):
    global anchox, anchoy, pista_minx, pista_miny
    x = nodo[0]
    y = nodo[1]
    px = pista_minx + x * anchox + anchox / 2
    py = pista_miny + y * anchoy + anchoy / 2
    return (px, py)


def heuristic(nodo):
    global destino_g
    distx = (nodo[0] - destino_g[0]) ** 2
    disty = (nodo[1] - destino_g[1]) ** 2
    dist = np.sqrt(min(distx, disty))
    return int(dist)


def A_estrella():
    global mundo, ruta, raiz, destino_p, calculo
    
    print('holi, estoy calculando')
    print(destino_g)
    mundo[destino_g[0]][destino_g[1]] = 'G'
    rta = []
    pq = queue.PriorityQueue()
    pq.put((0, raiz))
    origen = {raiz: None}
    dist = {raiz: 0}
    while not pq.empty():
        actual = pq.get()
        nodo = actual[1]
        prioridad = actual[0]
        if nodo == destino_g:
            break

        siguientes = puntos(nodo)

        for p in siguientes:
            distancia = 1
            if (abs(p[0] - nodo[0]) + abs(p[1] - nodo[1])) > 1:
                distancia = np.sqrt(2)
            n_cost = dist[nodo] + distancia
            if p not in dist or n_cost < dist[p]:
                dist[p] = n_cost
                prioridad = n_cost + heuristic(p)
                pq.put((prioridad, p))
                origen[p] = nodo

    actual = destino_g
    while actual != raiz:
        rta.insert(0, actual)
        actual = origen[actual]

    rta.insert(0, raiz)
    ruta = rta
    for punto in ruta:
        x = punto[0]
        y = punto[1]
        mundo[x][y] = 'O'

    print_mundo()
    print (ruta)
    calculo = True


def print_mundo():
    global mundo
    s = mundo.shape
    alto = s[1]
    ancho = s[0]
    for y in range(1, alto):
        s = ''
        for x in range(0, ancho):
            s += mundo[x][-y]
        print s
    s = ''
    for x in range(0, ancho):
        s += mundo[x][0]
    print s


# inicializacion del thread
#threading.Thread(target=A_estrella).start()


# funcion que se encarga de ejecutarse el control del robot y mover las ruadas del robot
def controlador():
    global vel, archivo, tiempo, sefini
    global girar, tiempos, error, cov
    global x, y, xs, ys, calculo, saturacion, saturacion2
    global alpha, beta, rho, minvel
    global destino, terminoPintar
    global kp, ka, kb, eps
    global Empezo, mundo, rate
    global R, J1, J1i, J2, J1_alt, J2i, check_pos


    rospy.init_node('master_client_py', anonymous=True)
    pub1 = rospy.Publisher('deseo_izq', Float32, queue_size=1)
    pub2 = rospy.Publisher('deseo_der', Float32, queue_size=1)
    pub3 = rospy.Publisher('empezar__adivinar', Bool, queue_size=1)
    pub4 = rospy.Publisher('robot_position', Pose, queue_size = 10)
    pub5 = rospy.Publisher('robot_uncertainty', Covariance, queue_size = 10)
    key = 0
    # Se suscribe a los topicos con la informacion del robot
    rospy.Subscriber('motor_vel', Float32MultiArray, posCallback)
    vel_deseada.data = [0, 0]
    try:

        Rate = rospy.Rate(rate)  # 10hz
        final = tiempo
        i = 0
        destinot = [0, 0, 0]
        inver = False
        grupo = 14
        ipaddr = "157.253.210.30"
        ack = ackService()
        print(ack)
        pose = Pose()
        cova = Covariance()
        
        pose.position.z = 0
        print('holi')
        while not rospy.is_shutdown():
            if calculo:
                #cleaprint('ahora si a movernos')
                dx = x - destinot[0]
                dy = y - destinot[1]
                rho = np.sqrt((dx) ** 2 + (dy) ** 2)
                alpha = - angulo + math.atan2(-dy, -dx)
                beta = - angulo - alpha
                if rho > eps:
                    v = kp * rho
                    bet2 = np.radians(destinot[2]) + beta
                    if bet2 < np.radians(-180):
                        bet2 = bet2 + np.radians(360)
                    if bet2 > np.radians(180):
                        bet2 = bet2 - np.radians(360)

                    if alpha < np.radians(-180):
                        alpha = alpha + np.radians(360)
                    if alpha > np.radians(180):
                        alpha = alpha - np.radians(360)


                    w = ka * alpha + kb * (bet2)
                    Xi = v * np.cos(angulo)
                    Yi = v * np.sin(angulo)
                    Si = np.array([Xi, Yi, w])

                    R = np.array([[np.cos(angulo), np.sin(angulo), 0], [-np.sin(angulo), np.cos(angulo), 0], [0, 0, 1]])


                    fi = np.dot(J2i, np.dot(J1_alt, np.dot(R, Si)))
                    rapidez = np.linalg.norm(fi)
                    if rapidez > saturacion:
                        fi = (fi / float(rapidez)) * saturacion
                    if i < len(ruta) and rapidez < minvel:
                        fi = (fi / float(rapidez)) * minvel
                    vel_deseada.data[0] = fi[0]
                    vel_deseada.data[1] = fi[1]

                elif i < len(ruta):
                    punto = ruta[i]
                    punto2 = cartesianas(punto)
                    puntoa = cartesianas((destinot[0], destinot[1]))
                    ang = 0
                    if i < len(ruta) - 2:
                        punto3 = cartesianas(ruta[i + 1])
                        dx2 = punto3[0] - punto2[0]
                        dy2 = punto3[1] - punto2[1]
                        if dx2 != 0 or dy2 != 0:
                            ang = math.atan2(dy2, dx2)

                    if i == len(ruta) - 1:
                        ang = destino[2]

                    destinot = [punto2[0], punto2[1], np.degrees(ang)]
                    print(destinot, inver)
                    i = i + 1   

                else:
                    print(np.radians(destinot[2]) + beta, np.degrees(angulo))
                    sefini = True
                    vel_deseada.data = [0, 0]
            
            pub1.publish(vel_deseada.data[0])
            pub2.publish(vel_deseada.data[1])
            pub3.publish(sefini)

            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = angulo
            pub4.publish(pose)

            cova.sigma11  = cov[0][0]
            cova.sigma12  = cov[0][1]
            cova.sigma13  = cov[0][2]

            cova.sigma21  = cov[1][0]
            cova.sigma22  = cov[1][1]
            cova.sigma23  = cov[1][2]

            cova.sigma31  = cov[2][0]
            cova.sigma32  = cov[2][1]
            cova.sigma33  = cov[2][2]
            pub5.publish(cova)

            Rate.sleep()


    except rospy.ServiceException as e:
        print(e.message)

                                                    

if __name__ == '__main__':

    #rospy.init_node('master_client_py', anonymous=True)
    masterReq = rospy.Service('start_service',StartService,start_service)
    try:
        controlador()
    except rospy.ROSInterruptException:
        pass

