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
Agente_config.connect()
time.sleep(1)
#Lo enciende y espera 5 segundos, esto para dejar que se inicialice correctamente el Pololu, Camara y Manipulador
Agente_config.on()
time.sleep(2)

Agente_config.arm_coords(60,60)
time.sleep(4)
Agente_config.arm_coords(60,80)
time.sleep(4)
Agente_config.arm_coords(80,80)
time.sleep(4)
Agente_config.arm_coords(80,60)
time.sleep(4)
Agente_config.arm_coords(50,50)
time.sleep(4)
#Al encenderse el agente, este coloca el manipulador en las posiciones base por default.

#Se desconecta del agente sin apagar, asi los servomotores conservan la posicion.
Agente_config.disconnect()
