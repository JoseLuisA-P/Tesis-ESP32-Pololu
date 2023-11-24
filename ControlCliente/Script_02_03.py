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

#Se utiliza para temporizar la toma de fotogramas
actual_time = time.time()
n = 0
capture = 0

#Durante 20 segundos se toma video de lo que ve el agente y se imprime en pantalla
while(n < 200):
    elapsed_time = time.time()
    diff = elapsed_time - actual_time
    
    if(elapsed_time > 0.1):
        capture = 1
        n = n + 1
        
    if(capture):
        raw_image = agent1.get_image()
        if(len(raw_image) >= 4700):
            processed_image = raw_image.reshape(60,80)
            h.set_data(processed_image)
            draw(), pause(0.05)
            capture = 0
        else:
            agent1.disconnect()
            time.sleep(0.1)
            agent1.connect()
            
plt.close()
agent1.off()
agent1.disconnect()