Las funciones utilizadas para configurar este proyecto se pueden agrupar en grupos, cada uno encargado de configurar un módulo y agregar propiedades para realizar tareas específicas.

**Configuración del Módulo WiFi:**
- `wifi_init_softap()`: Inicializa y configura el módulo WiFi como punto de acceso (AP).
- `nvs_init()`: Inicializa el sistema de almacenamiento no volátil y lo actualiza a la versión más actual al cargar el firmware.
- `wifi_event_handler()`: Se encarga de manejar los eventos que se llevan a cabo en la red WiFi; además, imprime los eventos para facilitar las pruebas.

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
