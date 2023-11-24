## Tabla de contenido:
- [Contenido de esta carpeta](#Contenido)
- [Requerimientos del sistema](#Sistema)
- [Clase para controlar el agente](#funciones)
- [Compatibilidad con otros lenguajes](#Compatibilidad)

## Contenido de esta carpeta

En esta carpeta se encuentra un conjunto de scripts creados para controlar el agente mediante una conexión WiFi.

En el archivo "FuncionesControlPololu.py", se ha creado la clase **PololuAgent**. Esta clase permite controlar al agente de manera sencilla mediante métodos que modifican las variables de control del agente y le hacen interactuar con su entorno.

Los archivos que comienzan con "Test_" en su nombre son pruebas realizadas para explorar las capacidades del agente, su tiempo de respuesta y la repetibilidad de sus instrucciones.

Los archivos que comienzan con "Script_" son referenciados en el manual de ensamble y sirven para realizar pruebas durante el ensamble del agente.

Por último, los archivos que comienzan con "Rutina_" se utilizan para inicializar o probar una configuración en el agente, colocándolo en posiciones iniciales o condiciones de prueba requeridas en su ensamble.

## Requerimientos del sistema

Se requiere de un sistema operativo con Python 3.x y con capacidad de conectarse por WiFi.

## Clase para controlar el agente

La clase para controlar el agente se encuentra en el script "FuncionesControlPololu.py". Esta clase permite controlar al agente y sus módulos de manera simple, sin limitar la profundidad de su control, ya que a los módulos se les pueden enviar valores en bruto para su control.

Para crear el objeto, se requiere la dirección IP del host, en este caso, la dirección de la placa al conectarse, y el puerto que se utilizará para la conexión. Al crear el objeto, se asignan en atributos privados la dirección IP, el puerto utilizado, se crea un buffer interno para el manejo de mensajes, y se crean las variables de odometría y de la batería del agente.


### Descripcion de los metodos
- `connect()`: Crea la conexión con el agente y el socket TCP a utilizar.
- `disconnect()`: Se desconecta del agente, elimina el socket creado y, si es necesario, limpia el buffer.
- `on()`: Envía el carácter de control para encender al agente y sus módulos, además, enciende la bandera para la adquisición de imágenes desde la cámara.
- `off()`: Envía el carácter de control para apagar al agente y sus módulos, y apaga la bandera para la adquisición de imágenes desde la cámara.
- `arm_raw_config()`: Envía el carácter de control para indicar al agente que debe actualizar la configuración del manipulador serial. Junto con el carácter, se envían en grados las configuraciones de las juntas del manipulador.
- `arm_coords()`: Envía el carácter de control para indicar al agente que debe actualizar el manipulador a una coordenada solicitada. Junto con el carácter de control, se envían las coordenadas para posicionar el efector final del manipulador.
- `battery_update()`: Envía el carácter de control para solicitar el nivel de la batería, recibe el dato en bruto obtenido por el ADC del TinyS3 y luego actualiza los atributos privados del voltaje y porcentaje de batería.
- `get_battery_voltage()` y `get_battery_level()`: Retornan como flotante el valor de voltaje o porcentaje de batería obtenidos.
- `set_motor_speed()`: Recibe los valores de velocidad de las ruedas izquierda y derecha como flotantes, los agrega al buffer interno del objeto junto con el carácter de control para indicar que las velocidades se deben actualizar y los envía al agente como un arreglo de bytes.
- `get_image()`: Envía el carácter de control al agente para solicitar la imagen y recibe 4800 bytes que representan el último fotograma guardado por el agente. Los agrega a un arreglo de NumPy y retorna este arreglo.
- `get_odo()`: Envía el carácter de control al agente para la solicitud de la odometría interna y recibe 12 bytes. Luego, convierte estos bytes en flotantes y los agrega a las variables internas del agente.


### Como utilizar esta clase

Previo a utilizar estos scripts es necesario conectarse a la red wifi del agente, 

**Importando la libreria y creando el objeto**
```python
from FuncionesControlPololu import PololuAgent

HOST = "192.168.4.1" #Direccion IP del host, TinyS3
PORT = 3333 #El puerto que se le va asignar al agente

agent1 = PololuAgent(HOST,PORT) #Creando el objeto con las variables
```

Para importar la librería al script que se está modificando en el momento, basta con tener el script "FuncionesControlPololu.py" en la misma carpeta e importar "PololuAgent". Con esta clase importada, se establece la dirección IP para conectarse con el agente y se asigna el puerto deseado, en este caso, se utilizó 3333 por conveniencia. Al establecer la dirección y el puerto, se puede crear el objeto pasando estas variables.

**Como conectarse y encender el agente**
```python
agent1.connect()    #Conectarse al agente
agent1.on()         #Encender al agente y sus módulos.
time.sleep(5)       #Este delay es de seguridad, asegura que todos los sistema se inicialicen correctamente
```

Con el objeto creado, se puede utilizar **connect()** para realizar la conexión con el socket. En caso de fallar la conexión, el manejo de errores al crear el socket detiene la ejecución del código e indica en la terminal el motivo por el cual no se pudo realizar la conexión. Al estar conectado al agente, se puede encender junto con sus módulos con **on()** y luego esperar de 5 a 10 segundos. Este delay es por seguridad y permite que tanto el agente, como la cámara y el manipulador serial terminen de configurarse.

**Como apagar al agente y desconectarse**
```python
agent1.off()            #Apagar al agente y sus módulos.
agent1.disconnect()     #Desconectarse del agente
```

Al terminar la ejecucion del programa se puede apagar al agente con **off()**, esto apaga al agente y sus modulos, luego se puede desconectar con **disconnect()**, lo que libera el socket del agente y permite que este acepta otra conexion a futuro. Previo a apagar el agente se recomienda dejarlo en reposo, velocidades de 0, y el manipulador serial en la posicion de reposo presentada en el manual, esto para asegurar que al encerse no se mueva de manera descontrolada o se arruine el manipulador.

***Las siguientes aplicaciones requieren al agente estar encendido y al cliente conectado a la red***

**Como mover al agente actualizando los valores de velocidades de sus llantas**

```python
#Establecer las velocidades para las llantas
velocidad_izquierda = 10.0
velocidad_derecha = 10.0

agent1.set_motor_speed(velocidad_izquierda,velocidad_derecha) #Enviar estas velocidades al agente para actualizarlas
```

Si se busca mover el agente, se puede enviar directamente la velocidad de cada llanta de manera individual. Para ello, basta con establecer su valor como flotante y luego, con **set_motor_speed()**, colocar estas velocidades. Esto enviará las velocidades al agente y actualizará su valor. Estas velocidades también pueden ser calculadas por un algoritmo de control, como un PID, y luego, pasar el resultado de la iteración al método para actualizar al agente.

**Como mover el manipulador serial**
```python
#Configuracion en grados de cada una de las juntas del manipulador
junta_1 = 90
junta_2 = 90
efector = 180

#Pasar estos valores al metodo para actualizar la configuracion en el agente
agent1.arm_raw_config(junta_1,junta_2,efector)
```

Para mover el manipulador con mayor precisión o después de utilizar un algoritmo de cinemática inversa, se puede utilizar **arm_raw_config()**. Este método recibe la configuración de cada junta en grados y la envía al agente para actualizar físicamente el manipulador.

```python
#Coordenadas para mover el efector final
coordenadaX = 50
coordenadaY = 50

#Enviar las coordenadas para posicionar el manipulador
agent1.arm_coords(coordenadaX,coordenadaY)
```

Para mover el efector final del manipulador a una posición coordenada específica, se utiliza **arm_coords()**. Este método recibe las coordenadas para mover el efector y actualiza su configuración internamente.

**Obtener la informacion de la celda**

```python
agent1.battery_update() #actualiza el valor de la bateria al ultimo leido por el agente

print(agent1.get_battery_voltage())     #Imprime el voltaje de la bateria
print(agent1.get_battery_level())       #Imprime el porcentaje restante de carga de la bateria
```

Si se requiere saber el nivel de batería del agente, primero se deben actualizar los atributos del objeto con **battery_update()**. Este método solicita al agente el último valor leído de la celda y lo envía al cliente. Con este valor actualizado, se puede obtener el voltaje de la celda con **get_battery_voltage()** o el porcentaje de carga restante con **get_battery_level()**. Al retornar un valor los últimos dos métodos, se pueden utilizar para actualizar una variable de control.

**Obtener informacion de la odometria interna del agente**

```python
agent1.get_odo()        #Obtener la odometria del agente
agent1.odoX             #Acceder a la coordenada X de la odometria
agent1.odoY             #Acceder a la coordenada Y de la odometria
agent1.odoTheta         #Acceder al ángulo Theta de la odometria
```

Para obtener la configuración del agente por medio de odometría interna, se puede utilizar **get_odo()**. Este método solicita el valor actualizado de odometría del agente y lo envía al cliente. Al haber actualizado la odometría del agente en el objeto, se puede acceder a sus valores utilizando **.odoX**, **.odoY** o **.odoTheta**. Las coordenadas están en metros y el ángulo está en radianes.

**Obtener el ultimo fotograma desde el agente y graficarlo con Matplolib**
```python
raw_image = agent1.get_image()                  #Obtener el fotograma como un vector fila de bytes
processed_image = raw_image.reshape(60,80)      #Cambiar sus dimensiones por una matriz de 60X80
fg = plt.figure()                               #Crear la figura
ax = fg.gca()                                   #Crear el objeto para los ejes
fg.show()                                       #Mostrar la figura
h = ax.imshow(processed_image, cmap = 'gray')   #Actualizar el valor de la figura con el arreglo y colocarlo en blanco y negro
plt.show()                                      #Mostrar la imagen
```

Para obtener el último fotograma obtenido con la cámara, se puede utilizar **get_image()**. Este método solicitará al agente el último fotograma obtenido y lo enviará al cliente como un vector fila de bytes. Luego, con este valor en bruto, se puede redimensionar con NumPy, crear una figura y graficarlo con Matplotlib para visualizar lo que ve el agente.

```python
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
```

En caso se requiera hacer un stream de la imagen desde el agente, se puede emplear el código previo. Este solicita la imagen y al recibir correctamente el fotograma, actualiza la imagen a una tasa de 10 FPS. En caso de fallar, se desconecta del agente para limpiar el buffer y luego solicitar de nuevo la imagen.


***Si se necesita utilizar multiples acciones del agente en conjunto, en los scripts incluidos en esta carpeta se pueden ver multiples ejemplos***

## Compatibilidad con otros lenguajes

Dado que esta clase se escribió utilizando la menor cantidad de librerías en Python, utiliza estructuras para la conversión entre bytes y tipos de datos, y utiliza sockets para conectarse con el agente. Virtualmente, se puede migrar a otros lenguajes que tengan estas funcionalidades con baja dificultad, siempre conservando la baja latencia.

Entre los lenguajes recomendados para su migración están:
- Matlab
- Java
- C++



