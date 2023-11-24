import sensor, image, pyb
from pyb import SPI

RED_LED_PIN = 1

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)

handshake = pyb.Pin('P4',pyb.Pin.IN)
pyb.LED(RED_LED_PIN).on()
sensor.skip_frames(time=1000)

interface = pyb.SPI(2, SPI.MASTER,baudrate=40*1000*1000, polarity = 0, phase = 0)
cs_pin = pyb.Pin("P3",pyb.Pin.OUT_PP) #Es necesario activar el pin manualmente

comprate = 10

#envio = sensor.snapshot().compressed(comprate).bytearray()
envio = sensor.snapshot()
print(envio.size())
#print(len(envio))

while(1):
    pyb.LED(RED_LED_PIN).off()
    if(handshake.value() == 0):
        #pyb.LED(RED_LED_PIN).on()
        #envio = sensor.snapshot().compressed(comprate).bytearray()
        envio = sensor.snapshot()
        #print(len(envio))
        cs_pin.low()
        interface.send(envio)
        cs_pin.high()
        #print("Enviado")
