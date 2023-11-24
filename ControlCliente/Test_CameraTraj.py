import matplotlib.pyplot as plt
from matplotlib.pyplot import draw, pause
import numpy as np
import socket
import math
import time
import struct
import io
from PIL import Image
from scipy.io import loadmat
from FuncionesControlPololu import PololuAgent

HOST = "192.168.4.1" #Para conectar  a la placa
PORT = 3333 #El puerto que se le va asignar al agente

agent1 = PololuAgent(HOST,PORT)

agent1.connect()

agent1.on()
time.sleep(10)

#Variables fisicas del agente
r = 32/(2*1000)
l = 96/(2*1000)

#Variables de control del agente
kpO = 10
kiO = 0.001;
kdO = 0.0
EO = 0.0
eO_I = 0.0

vo =  8.0
alpha = 0.95

raw_image = agent1.get_image()
processed_image = raw_image.reshape(60,80)
fg = plt.figure()
ax = fg.gca()
fg.show()
h = ax.imshow(processed_image, cmap = 'gray')

# x_i = []
# y_i = []
# n = 50
# x1 = np.linspace(0,2*math.pi,n)
#  
# radius = 0.5 
#  
# for i in x1:
#     testX = radius*math.cos(i)
#     testY = radius*math.sin(i)
#     x_i.append(testX)
#     y_i.append(testY)

puntos = loadmat('TryPath.mat')
x_i = puntos['AgentPathX']
y_i = puntos['AgentPathY']

n=100
P = 0

xg = x_i[P]
yg =y_i[P]

agent1.get_odo()

xActual = agent1.odoX
yActual = agent1.odoY

e = [xg-xActual, yg-yActual]
error = np.linalg.norm(e)
time.sleep(0.1)

capture = 0
actual_time = time.time()
flush_buffer = bytearray(1024)

while (P<(n-1) or error > 0.2):
    agent1.get_odo()  
    xg = x_i[P]
    yg =y_i[P]
    xActual = agent1.odoX
    yActual = agent1.odoY
    thetaActual = agent1.odoTheta
    
    e = [xg-xActual, yg-yActual]
    error = np.linalg.norm(e)
    thetag = math.atan2(e[1],e[0])
    
    if error<0.2 and P<(n-1):
        P = P + 1
    
    eP = error
    eO = thetag - thetaActual
    eO = math.atan2(math.sin(eO),math.cos(eO))

    power = -alpha*math.pow(eP,2)
    kP = vo*(1-math.exp(power))
    v = kP
    
    eO_D = eO - eO_I
    EO = EO + eO
    w = kpO*eO + kiO*EO + kdO*eO_D
    eO_I = eO
    
    vRight = (v + w*l)/r
    vLeft = (v - w*l)/r
    
    elapsed_time = time.time()
    diff = elapsed_time - actual_time
    
    if(diff > 0.4):
        capture = 1
        actual_time = time.time()
    
    if (capture  >  0):
        raw_image = agent1.get_image()
        if(len(raw_image) >= 4700):
            print(len(raw_image))
            processed_image = raw_image.reshape(60,80)
            h.set_data(processed_image)
            draw(), pause(0.05)
            capture = 0
        else:
            agent1.set_motor_speed(0.0,0.0)
            agent1.disconnect()
            time.sleep(0.2)
            agent1.connect()
    
    agent1.set_motor_speed(vLeft,vRight)
    
# agent1.set_motor_speed(0.0,0.0)
# 
# agent1.set_motor_speed(-30.0,30.0)
# time.sleep(5)
# agent1.set_motor_speed(30.0,-30.0)
# time.sleep(5)
# 
# agent1.set_motor_speed(0.0,0.0)

agent1.set_motor_speed(0.0,0.0)
time.sleep(2)
agent1.set_motor_speed(30.0,-30.0)
time.sleep(2.3)
agent1.set_motor_speed(0.0,0.0)
time.sleep(1)
agent1.set_motor_speed(-30.0,-30.0)
time.sleep(3.5)
agent1.set_motor_speed(0.0,0.0)
time.sleep(1)

plt.close()
agent1.off()

agent1.disconnect()