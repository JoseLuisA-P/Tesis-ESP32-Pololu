import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import socket
import time
import struct
import io
from PIL import Image
from FuncionesControlPololu import PololuAgent

HOST = "192.168.4.1" #Para conectar  a la placa
PORT = 3333 #El puerto que se le va asignar al agente

#Crea el objeto del agente para manipularlo
Agente_config = PololuAgent(HOST,PORT)
#Se espera 1 segundo para que se estabilice la conexion.
Agente_config.connect()
time.sleep(2)
#Lo enciende y espera 5 segundos, esto para dejar que se inicialice correctamente el Pololu, Camara y Manipulador
Agente_config.on()
time.sleep(3)

Agente_config.set_motor_speed(160.0,160.0)
time.sleep(2)
Agente_config.set_motor_speed(50.0,-50.0)
time.sleep(4)
Agente_config.set_motor_speed(-50.0,50.0)
time.sleep(4)
Agente_config.set_motor_speed(-160.0,-160.0)
time.sleep(2)
Agente_config.set_motor_speed(0.0,0.0)
time.sleep(2)

Agente_config.off()
Agente_config.disconnect()