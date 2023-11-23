# Desarrollo de una placa de expansión para los agentes Pololu 3Pi+ que expanda sus capacidades dentro del ecosistema Robotat.

Este trabajo se desarrolló con el objetivo de ampliar las capacidades de los agentes 3Pi+ utilizados en la plataforma de pruebas Robotat. Para aumentar las capacidades del agente, se implementó un manipulador serial de 3 grados de libertad, el cual utiliza servomotores como actuadores y contiene un efector final tipo garra. También, se implementó una cámara OpenMV Cam H7 que permite capturar desde el punto del agente un fotograma o una serie de fotogramas, para realizar un livestream del agente mientras realiza otras rutinas o capturar una imagen en un punto clave de la ejecución de una rutina. Por último, se implementó un algoritmo para el estacionamiento automático del agente, el cual permite el retorno del agente a una posición establecida mientras evita los obstáculos en la plataforma, lo que permite automatizar el proceso de pruebas y asegurar un correcto funcionamiento del agente. Estas expansiones y el agente son controlados desde un TinyS3, el cual dota al agente y sus módulos con la capacidad de ser controlado por medio de una conexión WiFi, desde un cliente remoto con la capacidad de expandirse a múltiples lenguajes al utilizar instrucciones sencillas para su control.

<center><img src="https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/TitlePic-PhotoRoom.png-PhotoRoom.png?raw=true" width="600" height="600"/></center>

Que contiene este repositorio:
- El firmware utilizado por la placa TinyS3.
- Los códigos para el control del agente y módulos desde Python.
- Manual para el ensamble de las placas de expansion y los modulos.
- Videos de las pruebas para el correcto ensamblaje del agente.
- Enlaces a las pruebas y validación del agente.

### Tabla de contenido:
- [Plataforma de trabajo](#plataforma)
- [Estructura del repositorio](#estructura)
- [Como ensamblar los modulos y placas](#ensamble)
- [Resultados](#Resultados)

### Plataforma de trabajo

![Platformio y ESPRESSIF](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/piolabs-espressif-partnership.png?raw=true)

El firmware del agente se desarrolló utilizando la plataforma PlatformIO en VSCode, junto con el framework de ESP-IDF desarrollado como una extensión por ESPRESSIF. La elección de VSCode para este proyecto se debe a su alta flexibilidad para configurar el entorno de desarrollo, desde las alertas visuales hasta el manejo de errores y las diversas extensiones para interactuar con módulos y bibliotecas. Además, su fácil integración con GitHub y la gestión de versiones permitieron un desarrollo más eficiente.

Se utilizó la extensión de PlatformIO debido a las diversas herramientas que contiene para programar y preparar los entornos de trabajo para diversas placas y microcontroladores. Esta extensión permite crear proyectos específicos para cada placa, configurar sus características de manera sencilla, seleccionar las versiones del lenguaje a utilizar y manejar los errores de compilación. En el lado físico, permite programar las placas desde VSCode, utilizar el monitor serial para leer los mensajes de error y configurar acorde al puerto y velocidad de comunicación con la placa. Además, facilita compilar el proyecto y verificar los errores al cargar datos a las placas.

El framework de ESP-IDF se eligió principalmente por la fácil integración de FreeRTOS, lo que permite aprovechar al máximo las capacidades de la placa y optimizar su rendimiento multitarea. Además, este framework permite utilizar las bibliotecas desarrolladas por ESPRESSIF para un manejo eficiente de los módulos de las placas basadas en ESP32, proporcionando mayor flexibilidad en su configuración y permitiendo el uso del multiplexado de los pines para los distintos módulos.

![Altium Designer](https://raw.githubusercontent.com/JoseLuisA-P/Tesis-ESP32-Pololu/main/assets/18189621-3f7a-435f-8a02-462efb2cec41.avif)

Para el desarrollo de las PCBs, se utilizó Altium Designer 2021. Aunque este software permite el uso de librerías de objetos comunes, fue necesario agregar librerías personalizadas para cada uno de los componentes adquiridos localmente sin un modelo definido. La elección de Altium Designer se debe a la variedad de herramientas que ofrece para el desarrollo de PCBs y la facilidad para modificar e interconectar proyectos.

![Inventor](https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/autodesk-inventor-professiona%C3%B6-1280x720.jpg?raw=true)

En cuanto al diseño de elementos físicos, se llevó a cabo en Autodesk Inventor 2024. Al contar con una interfaz robusta e intuitiva para el diseño y aprovechando la licencia profesional proporcionada por la universidad, es una herramienta que he utilizado a lo largo de mi carrera. Esta elección permitió minimizar el tiempo de desarrollo gracias a la familiaridad con la herramienta.

### Estructura del repositorio

El repositorio se encuentra estructurado de la siguiente forma:
- En la carpeta **PruebasCOMS**, se encuentra el proyecto utilizado para configurar y crear el firmware del TinyS3.
- En la carpeta **assets** se encuentran las imágenes utilizadas para este repositorio.
- En la carpeta **Videos** se encuentran videos de las pruebas de ensamblaje de este trabajo.
- En la carpeta **Manufactura** se encuentran los archivos para manufacturar las PCBs e imprimir las piezas de este trabajo.
- En la carpeta **ControlCliente** se encuentran los scripts para el control del agente desde Python.

### Como ensamblar los modulos y placas

Para ensamblar los módulos y placas utilizados en este trabajo, puede hacer referencia al documento **"ManualDeEnsamble.pdf"**. En este documento, encontrará una breve descripción de las placas, así como instrucciones detalladas sobre cómo ensamblar las placas y módulos. Además, se proporciona información sobre cómo montarlos y detalles sobre otros materiales adicionales que podrían ser necesarios en el proceso.

En el proceso de ensamblaje, se incluyen referencias a pruebas creadas para observar el correcto funcionamiento del agente antes de desplegarlo, lo que facilita el diagnóstico durante su ensamblaje.

### Resultados

En esta carpeta se incluye un listado de enlaces a videos de YouTube sin listar, en estos se puede observar el comportamiento del agente y las diversas pruebas realizadas. Las pruebas realizadas incluyen tanto los modulos de manera individual como la integracion de los multiples modulos operando al mismo tiempo.
