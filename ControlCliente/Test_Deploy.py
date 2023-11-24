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
# 
# agent1.set_motor_speed(70.0,-90.0)
# time.sleep(1)
# agent1.set_motor_speed(60.0,60.0)
# time.sleep(2)
# 
# agent1.set_motor_speed(0.0,0.0)
# time.sleep(1)

agent1.arm_raw_config(0,180,180)
time.sleep(10)

agent1.off()
agent1.disconnect()