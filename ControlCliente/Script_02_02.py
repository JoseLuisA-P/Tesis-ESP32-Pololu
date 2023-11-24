import matplotlib.pyplot as plt
from matplotlib.pyplot import draw, pause
from matplotlib.animation import FuncAnimation
import numpy as np
import socket
import math
import time
import struct
import io
from PIL import Image
from FuncionesControlPololu import PololuAgent

HOST = "192.168.4.1" #Para conectar  a la placa
PORT = 3333 #El puerto que se le va asignar al agente

agent1 = PololuAgent(HOST,PORT)

agent1.connect()

agent1.on()
time.sleep(5) #Este delay es de seguridad, asegura que todos los sistema se inicialicen correctamente

#Solicita la imagen, la obtiene, le da el formato requerido y la muestra
raw_image = agent1.get_image()
processed_image = raw_image.reshape(60,80)
fg = plt.figure()
ax = fg.gca()
fg.show()
h = ax.imshow(processed_image, cmap = 'gray')
#Al mostrar la imagen espera que se cierre la pestaña para apagar al agente
plt.show()

agent1.off()
agent1.disconnect()

