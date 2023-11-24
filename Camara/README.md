El script presentado en esta carpeta contiene la configuración requerida para el envío bajo demanda de los fotogramas capturados por la cámara. La configuración de la cámara y resolución se pueden modificar acorde a los requerimientos del proyecto para procesar la información necesaria. Se pueden agregar más eventos en el bucle while para que la cámara sea capaz de procesar datos, disminuir la carga en el agente, o realizar un análisis de la imagen y enviar los datos.

Para poder enviar los datos, se debe utilizar la siguiente estructura:

```python
if(handshake.value() == 0):
    envio = sensor.snapshot()
    cs_pin.low()
    interface.send(envio)
    cs_pin.high()
```

En esta condición if, se envía el fotograma al agente. Para que esto suceda, el fotograma debe estar en blanco y negro con una resolución de 60x80 bytes. Si se desea enviar otros datos por SPI desde la cámara, se puede modificar este bucle if y enviar la estructura de datos requerida. Sin embargo, esto requiere modificar la función **AskForPicture()** en el firmware del TinyS3 para que reciba el mensaje del tamaño solicitado y lo procese acorde a lo requerido.

