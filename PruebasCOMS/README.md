## Tabla de contenido:
- [Acerca de este proyecto](#Acerca-de-este-proyecto)
- [Requerimientos del proyecto](#Requerimientos-del-proyecto)
- [Preparando el entorno](#Preparando-el-entorno)
- [Configuracion de la placa](#Configuracion-de-la-placa)
- [Breve descripcion de las funciones](#Breve-descripcion-de-las-funciones)

## Acerca de este proyecto
Este proyecto se creó para el desarrollo del firmware a utilizar en el TinyS3, el cual es el módulo central de control del agente robótico. Este dota al agente de la capacidad de comunicarse con clientes a través de WiFi, utilizando una lógica simple para el envío y recepción de datos. Además, esta placa permite controlar los módulos del manipulador serial y la cámara montada en el agente, tanto de manera individual como en paralelo, al fijar el envío de instrucciones en un orden específico que se presenta posteriormente.

Se desarrolló con el framework de ESP-IDF para el uso de las librerías creadas por ESPRESSIF para las placas basadas en el chip ESP32. Además, se utilizó para integrar FreeRTOS en el proyecto y mejorar el rendimiento del agente en actividades multitarea.

En este contexto, FreeRTOS dota al agente con un pseudo sistema operativo, el cual permite al agente responder a instrucciones enviadas desde un cliente, actualizar sus variables de control y actualizar el estado de sus módulos, con muy baja latencia. Esto es ideal para agentes robóticos en plataformas de desarrollo y estudio, ya que amplía la posibilidad de implementarlo en algoritmos de enjambre.

## Requerimientos del proyecto

Este proyecto puede ser modificado en Windows, MacOS o cualquier distribución de Linux capaz de ejecutar VSCode y PlatformIO. Además, el equipo debe ser capaz de ejecutar Python 3.x y Matlab R2021a en adelante. Estos requisitos son necesarios para poder ejecutar las funciones del cliente, interactuar con el robotat y tener la capacidad de modificar el proyecto.


## Preparando el entorno

Para instalar VSCode, se puede seguir el siguiente [enlace](https://code.visualstudio.com/docs/setup/setup-overview). Este enlace proporciona una visión general de VSCode, y se debe de seleccionar el sistema operativo que se esta utilizando. Se recomienda tener GitHub instalado e integrarlo en el entorno de VSCode, lo que facilitará, en el futuro, la actualización de la información del proyecto al contar con un punto de retorno seguro y comprobado, ademas de un eficiente manejo de versiones.

Una vez que se ha instalado VSCode en el sistema, es necesario instalar la extensión de PlatformIO, lo cual se puede realizar siguiendo las instrucciones del siguiente [enlace](https://platformio.org/install/ide?install=vscode).

Con la extensión de PlatformIO instalada, se debe de descargar este proyecto en su última versión para modificarlo, realizar un fork o clonarlo en un repositorio personal. Teniendo el proyecto disponible en tu equipo, puedes importarlo haciendo clic en **Open project** o **Abrir proyecto** en la página de inicio de PlatformIO.

![Importar el proyecto](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/Platformio1.drawio.png?raw=true)

Al hacer clic en este acceso rápido, se selecciona la carpeta donde se descargó el proyecto y, luego, se debe de hacer clic en abrir. Este proceso descargará las librerías requeridas por el proyecto, actualizará las librerías existentes y descargará los scripts necesarios para programar la placa. Este proceso suele tomar entre 5 y 25 minutos, y es importante que la computadora no entre en reposo durante este tiempo. Sin embargo, no es necesario mantener la aplicación abierta, por lo que se puede realizar otras actividades mientras se importa el proyecto.

![Como se ve el proyecto importado](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/ArchivoDescargado.png?raw=true)

Al finalizar la importación del proyecto, se agregará al workspace actualmente abierto o a un workspace sin nombre, como se muestra a la izquierda. Dentro de él, en la carpeta "src", encontrarás el archivo principal (main.c) del proyecto. Al abrirlo, verás la pantalla como se muestra en la imagen superior. Después de dar clic en compilar, se generará una versión lista para cargar en la placa y comenzar a trabajar. Con esto completado, el proyecto se ha importado correctamente y está listo para ser modificado.

En caso de que surjan errores durante la compilación o la importación, las advertencias de PlatformIO son explícitas y permiten resolver los errores al leerlos de manera adecuada. Si, después de seguir las indicaciones de PlatformIO, no puedes importar el proyecto, se recomienda descargarlo e instalarlo en el escritorio de tu equipo.

## Configuracion de la placa


Para configurar la placa al iniciar un proyecto nuevo o importar porciones del proyecto, puedes utilizar la siguiente configuración:

![Configuraion de la placa](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/ConfiguracionPlaca.png?raw=true)

La configuración presentada es capaz de ejecutar todas las librerías utilizadas y aprovechar al máximo las capacidades de la placa. Además, permite la lectura de distintos tipos de mensajes de la placa y los diferencia por color, haciendo más intuitivo el proceso de pruebas y desarrollo.


# Breve descripcion de las funciones
Las funciones utilizadas para configurar este proyecto se pueden separar en grupos, cada uno encargado de configurar un módulo y agregar propiedades para realizar tareas específicas.

**Configuración del Módulo WiFi:**
- `wifi_init_softap()`: Inicializa y configura el módulo WiFi como punto de acceso (AP).
- `nvs_init()`: Inicializa el sistema de almacenamiento no volátil y lo actualiza a la versión más actual al cargar el firmware.
- `wifi_event_handler()`: Se encarga de manejar los eventos que se llevan a cabo en la red WiFi, además, imprime los eventos para facilitar las pruebas.

**Configuración del Socket TCP:**
- `tcp_socket_init()`: Inicializa el socket TCP y crea la tarea del handler para manejar los mensajes y solicitudes del cliente.
- `handle_socket()`: Maneja los datos recibidos por el cliente y realiza acciones después de leer la variable de control.

**Comunicación con el Agente a través del Bus UART:**
- `UART_init()`: Configura el bus UART con los parámetros requeridos por el agente.
- `uart_tx()` y `uart_rx()`: Encargadas del envío y recepción periódico de datos al agente utilizando el bus UART.
- `decodeCBOR()`: Decodifica el contenedor de CBOR recibido en el bus y asigna los valores en este a las variables de odometría interna.
- `encodeCBOR()`: Codifica las velocidades de las ruedas del agente, agregándolas a un contenedor de CBOR.

**Comunicación con la Cámara a través del Bus SPI:**
- `SPI_init_config()`: Configura el módulo SPI, asigna los callbacks para las transmisiones y activa el pin de handshake.
- `my_pre_cb()` y `my_post_cb()`: Son los callbacks de la transmisión SPI y cambian el estado del pin de handshake para solicitar información a la cámara.
- `AskForPicture()`: Bucle que solicita bajo demanda la imagen desde la cámara y actualiza el buffer de la última imagen capturada.

**Manipulador Serial utilizando el Módulo MCPWM:**
- `mcpwm_initialize()`: Configura e inicializa los 3 canales de PWM.
- `updatePositionSCH()`: Actualiza los ciclos de trabajo de los canales de PWM de cada servomotor ante modificaciones realizadas por el cliente.
- `CalcConfig()`: Calcula la configuración y ciclo de trabajo requerido por los servomotores para colocarlos en una coordenada especificada por el cliente.

**Lectura de la Celda para la Alimentación del Agente:**
- `ADCConfig()`: Configura e inicializa el canal ADC en modo oneshot.
- `BatteryUpdate()`: Realiza lecturas periódicas al nivel de la batería y almacena el último valor leído en una variable.

Dado que el código utiliza envíos de datos al cliente bajo demanda y parte de estos datos son recabados de manera periódica al encender el agente, se utiliza `initSemaphores()` para inicializar y configurar las estructuras de control.
