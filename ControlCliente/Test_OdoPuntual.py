import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import socket
import time
import struct
import io
from scipy import interpolate
from PIL import Image
from FuncionesControlPololu import PololuAgent

HOST = "192.168.4.1" #Para conectar  a la placa
PORT = 3333 #El puerto que se le va asignar al agente

agent1 = PololuAgent(HOST,PORT)

agent1.connect()

agent1.on()
time.sleep(5)

#Variables fisicas del agente
r = 32/(2*1000)
l = 96/(2*1000)

#Variables de control del agente
kpO = 10
kiO = 0.001;
kdO = 0.0
EO = 0.0
eO_I = 0.0

vo =  5.0
alpha = 0.95


xg = 1.0
yg = 1.0

agent1.get_odo()

xActual = agent1.odoX
yActual = agent1.odoY

e = [xg-xActual, yg-yActual]
error = np.linalg.norm(e)
time.sleep(0.1)

while (error > 0.1):
    agent1.get_odo()
    xg = 1.0
    yg = 1.0
    xActual = agent1.odoX
    yActual = agent1.odoY
    print()
    thetaActual = agent1.odoTheta
    
    e = [xg-xActual, yg-yActual]
    error = np.linalg.norm(e)
    thetag = math.atan2(float(e[1]),float(e[0]))
    eP = error
    eO = thetag - thetaActual
    eO = math.atan2(math.sin(eO),math.cos(eO))

    kP = vo*(1-math.exp(-alpha*eP*eP))/eP
    v = kP*eP
    
    eO_D = eO - eO_I
    EO = EO + eO
    w = kpO*eO + kiO*EO + kdO*eO_D
    eO_I = eO
    
    vRight = (v + w*l)/r
    vLeft = (v - w*l)/r
    
    agent1.set_motor_speed(vLeft,vRight)
    
agent1.set_motor_speed(0.0,0.0)

agent1.off()
agent1.disconnect()
    
    


