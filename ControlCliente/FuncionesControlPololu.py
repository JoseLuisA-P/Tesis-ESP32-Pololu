import numpy as np
import socket
import time
import struct
import io

class PololuAgent:
    
    def __init__(self, ip_host, agent_port):
        self.ip_address = ip_host
        self.agent_port = agent_port
        self.buffer = bytearray(512)
        self.battery_voltage = 0
        self.battery_level = 0
        self.odoX = 0.0
        self.odoY = 0.0
        self.odoTheta = 0.0
        
    def connect(self):
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.socket.connect((self.ip_address,self.agent_port))
    
    def disconnect(self):
        self.socket.close()
    
    def on(self):
        self.buffer[0] = 0x43 #Para encenderlo
        self.socket.send(self.buffer)
    
    def off(self):
        self.buffer[0] = 0x44 #Para apagarlo
        self.socket.send(self.buffer)
     
    def arm_raw_config(self, config_joint1, config_joint2, config_endef):
        self.buffer[0] = 0x42 #Para enviar angulos en bruto
        self.buffer[1] = config_joint1
        self.buffer[2] = config_joint2
        self.buffer[3] = config_endef
        self.socket.send(self.buffer)
    
    def arm_coords(self, Xcoord, Ycoord):
        self.buffer[0] = 0x45 #Para enviar angulos en bruto
        self.buffer[1] = Xcoord
        self.buffer[2] = Ycoord
        self.socket.send(self.buffer)
    
    def battery_update(self):
        self.buffer[0] = 0x46 #Para solicitar el valor de la bateria
        self.socket.send(self.buffer)
        raw = self.socket.recv(4)
        level = raw[1]<<8|raw[0]
        self.battery_voltage = level*5.12/4096 #Ajuste para obtener una lectura mas precisa
        self.battery_level = self.battery_voltage*100/5.12
    
    def get_battery_voltage(self):
        return self.battery_voltage
    
    def get_battery_level(self):
        return self.battery_level
    
    def set_motor_speed(self,left_speed:float,right_speed:float):
        
        if (not isinstance(left_speed,float) or not isinstance(right_speed,float)):
            raise TypeError("Ambos valores deben de ser floats")
        else:
            pass
        
        byte_right = struct.unpack('<I', struct.pack('<f', right_speed))[0]
        byte_left = struct.unpack('<I', struct.pack('<f', left_speed))[0]
        
        self.buffer[0] = 0x47 #Indicar que son las velocidades
        
        for i in range(4):
            #Colocar los bytes de la velocidad en el mensaje
            self.buffer[i+1]=(byte_left>>(i*8)) & 0xFF
            self.buffer[i+5]=(byte_right>>(i*8))& 0xFF
            
        self.socket.send(self.buffer)
    
    def get_image(self):
        self.buffer[0] = 0x41
        self.socket.send(self.buffer)
        vector_image = self.socket.recv(4800)
        image_array = np.frombuffer(vector_image,dtype = np.uint8)
        return image_array
    
    def get_odo(self):
        self.buffer[0] = 0x48
        self.socket.send(self.buffer)
        FullOdo = self.socket.recv(12) #Se envian los 3 floats, por eso se reciben 12 bytes
        #Repartiendo el mensaje en bytes
        byteX = FullOdo[0:4]
        byteY = FullOdo[4:8]
        byteTheta = FullOdo[8:12]
        #Desempaquetando los bytes en floats
        self.odoX = struct.unpack('f',byteX)[0]/1000.0
        self.odoY = struct.unpack('f',byteY)[0]/1000.0
        self.odoTheta = struct.unpack('f',byteTheta)[0]
    
    def __str__(self):
        return f"Direccion ip conectado: {self.ip_address}  \nPuerto de conexion: {self.agent_port}"