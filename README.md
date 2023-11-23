# Desarrollo de una placa de expansión para los agentes Pololu 3Pi+ que expanda sus capacidades dentro del ecosistema Robotat.

Este trabajo se desarrolló con el objetivo de ampliar las capacidades de los agentes 3Pi+ utilizados en la plataforma de pruebas Robotat. Para aumentar las capacidades del agente, se implementó un manipulador serial de 3 grados de libertad, el cual utiliza servomotores como actuadores y contiene un efector final tipo garra. También, se implementó una cámara OpenMV Cam H7 que permite capturar desde el punto del agente un fotograma o una serie de fotogramas, para realizar un livestream del agente mientras realiza otras rutinas o capturar una imagen en un punto clave de la ejecución de una rutina. Por último, se implementó un algoritmo para el estacionamiento automático del agente, el cual permite el retorno del agente a una posición establecida mientras evita los obstáculos en la plataforma, lo que permite automatizar el proceso de pruebas y asegurar un correcto funcionamiento del agente. Estas expansiones y el agente son controlados desde un TinyS3, el cual dota al agente y sus módulos con la capacidad de ser controlado por medio de una conexión WiFi, desde un cliente remoto con la capacidad de expandirse a múltiples lenguajes al utilizar instrucciones sencillas para su control.

<img src="https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu/blob/main/assets/TitlePic-PhotoRoom.png-PhotoRoom.png?raw=true" width="800" height="800"/>

En este repositorio se encuentra:
- El firmware utilizado por la placa TinyS3.
- Los códigos para el control del agente y módulos desde Python.
- Videos de las pruebas para el correcto ensamblaje del agente.
- Enlaces a las pruebas y validación del agente.
 
