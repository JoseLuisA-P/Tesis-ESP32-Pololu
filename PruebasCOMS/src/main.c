/**
 * @file main.c
 * @author José Luis Alvarez Pineda (19392)
 * @date 13 Oct 2023
 * @brief Script principal para configurar la placa TinyS3 a utilizar en los agentes 3Pi+
 * de la plataforma de pruebas del Robotat UVG.
 *
 * En este script se configura el modulo WiFi, los modulos MCPWM, los pines de I/O,
 * los pines analogicos, el bus SPI y las rutinas de control para aumentar las capacidades del agente
 * 3Pi+.  
 * Este script es compatible con el manipulador serial y la OpenMV Cam H7, controlandolos por 
 * medio de instrucciones WiFi. Con este se puede controlar el manipulador serial al envair las 
 * coordenadas para mover su efector final y enviando las configuraciones en bruto del manipulador,
 * ademas de poder abrir el efector a distitos tamaños, los servomotores del manipulador son manejados 
 * con los pines de MCPWM configurados. La OpenMV Cam H7 se conecta por medio de SPI con una tasa de 
 * transferencia de 40Mbps full-duplex, lo que permite el envio de la imagen en blanco y negro 
 * por medio de WiFi bajo demanda, lo que permite capturar la imagen en un momento dado o realizar un
 * webstream que alcanza hasta 6 FPS en promedio. Por ultimo, este permite controlar las velocidades
 * de los motores del agente por medio de WiFi, al enviar las velocidades de cada motor, y tambien 
 * tiene la capacidad de encender o apagar todo los modulos con un solo comando.
 * 
 * @see https://github.com/JoseLuisA-P/Tesis-ESP32-Pololu.git
 */


//De cajon
#include <stdio.h>
#include <sys/fcntl.h>
//Relacionado a RTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
//Relacionada al UART
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
//Relacionado al Wifi
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
//Implementando el stack TCP-IP
#include "esp_system.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
//Manejo de Strings
#include <string.h>
//Manejo de los servos
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
//Relacionado al SPI
#include "driver/spi_slave.h"
#include "driver/spi_master.h"
//Codificando con CBOR
#include "../src/cbor.h"
//Operaciones trigonometrica
#include <math.h>
//Lectura con el ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//Parametros para la configuracion del servidor
#define SSID                "Pololu3Pi+"
#define WIFIPASSWORD        "MT30062023"
#define WIFICHANNEL         2
#define MAXCONNECTIONS      1

//Puerto para la comunicacion
#define PORT 3333
int sock;
int LisSock;
uint8_t tcpavail = 0;

//Parametros para configurar el timer del LEDC
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_10_BIT // Resolucion del ciclo de trabajo de 13 bits

//Parametros para configurar el canal y pin del servo con el LEDC
#define SERVO1_CHANNEL          LEDC_CHANNEL_0
#define SERVO1_PIN              GPIO_NUM_1
#define SERVO2_CHANNEL          LEDC_CHANNEL_1
#define SERVO2_PIN              GPIO_NUM_2
#define SERVO3_CHANNEL          LEDC_CHANNEL_2
#define SERVO3_PIN              GPIO_NUM_3

//Parametros para calcular el duty en base al angulo
const uint32_t minDeg = 0;
const uint32_t maxDeg = 180;
const uint32_t minDuty = 500;
const uint32_t maxDuty = 1000;
uint32_t dutyValue;

//Configuracion de pines para el SPI
#define SPI_NAME    SPI2_HOST
#define SPI_MISO    GPIO_NUM_37
#define SPI_MOSI    GPIO_NUM_35
#define SPI_CLK     GPIO_NUM_36
#define SPI_CS      GPIO_NUM_34
#define SPI_HAND    GPIO_NUM_9

//PIN para encender el pololu
#define POLOLUSWITCH    GPIO_NUM_4

//Variables para las solicitudes
#define BUFFER_SIZE 4800 //antes 1600
char rx_buffer[BUFFER_SIZE];
//char recvbuf[3];
uint8_t SPIAsk = 0;
spi_device_interface_config_t CamH7;
spi_device_handle_t  SPIHandle;

//TAG para el log de errores o informacion
static const char *TAG = "Prueba";

//Control de los servos
#define SERVO_MIN_PULSEWIDTH 400  // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH 2400 // Maximum pulse width in microseconds
#define SERVO_MAX_DEGREE 180      // Maximum angle in degrees

//Configuracion del CBOR 
#define CBOR_BUF_SIZE       (32)
#define CBOR_MSG_MIN_SIZE   (16)

enum parser_fsm_state
{
    PARSER_START = 0,
    PARSER_IS_INSIDE_ARRAY,
    PARSER_GOT_X_POS,
    PARSER_GOT_Y_POS,
    PARSER_GOT_ANGLE
} parser_state;

uint8_t sendOdo = 0; //Para enviar la odometria al cliente
char odo_buffer[BUFFER_SIZE];
float xi_odo[3];  // Valores de [x,y,theta] del agente
uint8_t cbor_buf_tx[CBOR_BUF_SIZE] = {0};
uint8_t cbor_buf_rx[CBOR_BUF_SIZE] = {0};
static const unsigned int odoread_time_ms = 100;
CborEncoder encoder, array_encoder;
CborParser parser;
CborValue it, arr;
volatile float phi_ell = 0;//-50;  // in rpm
volatile float phi_r = 0;//50;     // in rpm

union{
    float input_flotante;
    uint8_t bytes[4];
} odoX, odoY, odoTheta; //Para convertir los floats en bytes y luego enviarlos al cliente

//Segunda configuracion del CBOR
float odo_x = 0;
float odo_y = 0;
float odo_theta = 0;

//Configuracion del UART
#define RX_BUF_SIZE         (1024)
#define UART_PORT           UART_NUM_2
#define TXD_PIN             (GPIO_NUM_7)
#define RXD_PIN             (GPIO_NUM_8)

static const unsigned int control_time_ms = 100;

/**
 * \struct
 * @brief Estuctura utilizada para conservar los parametros del manipulador serial.
 *
 * En esta estructura se incluyen los valores de configuracion de las juntas y su 
 * ciclo de trabajo.  Los valores de las juntas es el angulo que deben de mantener
 * acorde a la ultima configuracion realizada, con este valor se realizan los pasos 
 * para calcular el nuevo ciclo de trabajo y asi colocar la junta en esa configuracion,
 * ademas de utilizarse como el ultimo valor configurado.
 * El valor del ciclo de trabajo se utiliza para mantener la posicion luego de 
 * actualizar la configuracion de la junta y colocar el manipulador en su posicion
 * luego de un reinicio.
 * 
 */

typedef struct {
    uint32_t joint1;    /**< Configuracion de la junta 1, en grados. */
    uint32_t joint2;    /**< Configuracion de la junta 2, en grados. */
    uint32_t gripper;   /**< Configuracion de la junta 3, en grados. */
    uint32_t duty_us1;  /**< Ciclo de trabajo en la junta 1. */
    uint32_t duty_us2;  /**< Ciclo de trabajo en la junta 2. */
    uint32_t duty_us3;  /**< Ciclo de trabajo en la junta 3. */
}   ManSer;

ManSer brazo = {60,60,100,0,0,0};

uint32_t configj1,configj2,configj3;
uint32_t step1,step2,step3;

uint32_t q1, q2; //Configuraciones calculadas
uint32_t es1 = 47; //Largo eslabon 1
uint32_t es2 = 68; //Largo eslabon 2 
uint8_t coordX,coordY;//Configuraciones en XY para colocar el brazo
uint8_t updateCoords = 0;//Para actualizar la configuracion luego de enviar coordenadas

//Variables para el ADC
static int adc_raw[1][10]; //Valores en bruto del ADC
adc_oneshot_unit_handle_t adc1_handle; //Handler para las interrupciones
int batlevel = 0; //Flag para cuando se lee el nivel de bateria

//Prototipos para algunas funciones
static void AskForPicture(void *arg);

/*!
 \fn size_t cbor_encode_wheel_speeds(void)
 @brief Codifica las velocidades de la rueda en formato CBOR en un buffer.

 Inicializa el codificador CBOR con el buffer especificado y su tamaño, luego, se crea
 un array de CBOR con las velocidades de cada motor como un puntu flotante. Por ultimo, 
 se ciera el contenedor del array y se devuelve el tamaño del buffer para ser enviado.

*/

size_t cbor_encode_wheel_speeds(void)
{
    cbor_encoder_init(&encoder, cbor_buf_tx, sizeof(cbor_buf_tx), 0);
    cbor_encoder_create_array(&encoder, &array_encoder, 2);
    cbor_encode_float(&array_encoder, phi_ell);
    cbor_encode_float(&array_encoder, phi_r);
    cbor_encoder_close_container(&encoder, &array_encoder);

    return cbor_encoder_get_buffer_size(&encoder, cbor_buf_tx);
}

/*!
 \fn void UART_init(void)
 @brief Configura la interfaz UART.

 Configura los parametros del UART como la velocidad de transmision, tamaño del mensaje,
 paridad, bits de parada y el control de flujo entre transacciones. Se asignan los pines
 de TX y RX y se instala el controlador, inicializandolo para que opere en transacciones.
 
 \note El buffer de TX puede o no ser configurado, esto dependera del tamaño del mensaje.

*/

void UART_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };

    uart_driver_install(UART_PORT , RX_BUF_SIZE, 0, 0, NULL, 0);
    uart_param_config(UART_PORT , &uart_config);
    uart_set_pin(UART_PORT , TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);   
}

/*!
 \fn static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
 @brief Handler del modulo WiFi.

 Despliega informacion sobre los eventos que ocurren luego de crear configurar el modulo WiFi y el Access Point,
 desplegando la direccion MAC e IP de los clientes al conectarse o desconectarse.
 
 \param event_base Define la variable utilizada para registrar el evento.
 \param event_id En este se indica el id del evento a registrar o si se deben de registrar todos los eventos.
 \param event_data Es la informacion de respuesta generada ante el evento registrado.
 \return En el monitor serial el estado de la conexion. 
 \note Este handler se utiliza en la funcion "esp__event_handler_instance_register", no debe de ser utilizado de
 manera directa en el codigo. 
*/

static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    switch(event_id)
    {
        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",MAC2STR(event->mac), event->aid);
            break;   
        }
        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",MAC2STR(event->mac), event->aid);
            break;
        }
        default:
            break;

    }
}

/*!
 \fn void wifi_init_softap(void)
 @brief Inicializa y configura el modulo WiFi como Access Point (SoftAP)

 Configura el modulo WiFi como un Access Point utilizando el nombre de red (SSID) y contraseña (WIFIPASSWORD),
 este se configura utilizando el protocolo de seguridad WPA2-PSK, manteniendo a la red segura. Al finalizar
 la configuracion despliega un mensaje con los parametros necesarios para la conexion a la red.
 
 \note Solo se debe de llamar una vez y esto luego de haber inicializado la NVS
 \see void nvs_init(void)

*/

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&wifi_event_handler,NULL,NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SSID,
            .ssid_len = strlen(SSID),
            .channel = WIFICHANNEL,
            .password = WIFIPASSWORD,
            .max_connection = MAXCONNECTIONS,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                    .required = true,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",SSID, WIFIPASSWORD, WIFICHANNEL);
}

/*!
 \fn void nvs_init(void)
 @brief Inicializa el sistema de almacenamiento flash no volatil (NVS).

 Inicializa el sistema de almacenamiento flash no volatil (NVS), verificando la cantidad de 
 paginas libres en la flash y la ultima version utilizada para su configuracion. Si no hay suficientes
 paginas libres o al momento de cargar el codigo hay una nueva version, borra la flash y la
 configura para poder ser utilizada por otros modulos. Por ultimo, despliega en el monitor serial
 los posibles errores que se presenten al configurarla.

*/

void nvs_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);  
}

/*!
 \fn static void TCPSendRobust(void *arg)
 @brief Envio de datos por medio del Socket TCP.

 Envia un paquete de gran tamaño por medio del socket WiFi al ser activada una bandera, 
 utilizando el tamaño del Buffer como referencia. Envia el paquete por porciones, 
 utilizando el ancho de banda maximo cuando es posible. En caso de presentarse un 
 error durante el envio lo despliega en el monito serial y al terminar el envio 
 exitosamente se baja la bandera.

*/

static void TCPSendRobust(void *arg)
{
    while(1)
    {
        if(tcpavail>0)
        {
            printf ("TCPEnviado\n");
            int to_write = BUFFER_SIZE;
            int written = 0;

            while (to_write > 0) {
                    written = send(sock, &rx_buffer, to_write, 0);
                    //int written = send(sock, codedData, to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    tcpavail = 0;
                }
                to_write -= written;
            }
            
            tcpavail = 0;
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}

/*!
 \fn void handle_socket(void *pvParameters)
 @brief Handler para el manejo de los datos recibidos por WiFi.

 Utilizando el Socket creado para la conexion con el cliente, recibe un paquete de
 512 bytes utilizado en el handler. Con este paquete, toma la informacion del primer
 byte para determinar el tipo de tarea a realizar y como procesar el resto de bytes,
 ademas de cuales banderas activar para el proceso. En caso de recibir un paquete para 
 terminar la conexion, cierra el cliente y elimina la tarea creada.

 \param pvParameters Socket creado para la conexion con el cliente.

 \note Esta se utiliza como una tarea creada luego de terminar de configurar el Socket,
        para evitar errores de ejecucion.
*/

void handle_socket(void *pvParameters)
{
    int client_socket = (int)pvParameters;

    while(1) 
    {
        int len;
        char rx2_buffer[513];
        len = recv(client_socket, rx2_buffer, sizeof(rx2_buffer)-1, 0);

        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            break;

        } else if (len == 0) {
            ESP_LOGE(TAG, "Connection closed");
            break;

        } else {
            ESP_LOGI(TAG, "Received %d bytes", len);
            if(rx2_buffer[0]=='A') //Solicitar la imagen actual en la camara
            {
                SPIAsk = 1;
            }
            else if(rx2_buffer[0] == 'B') //Configurar los servos
            {
                configj1 = rx2_buffer[1];
                if(configj1 > 110)configj1 = 110;
                configj2 = rx2_buffer[2];
                if(configj2 < 45)configj2 = 45;
                else if(configj2 > SERVO_MAX_DEGREE)configj2 = SERVO_MAX_DEGREE;
                configj3 = rx2_buffer[3];
                if(configj3 < 40)configj3 = 40;
                else if(configj3 > SERVO_MAX_DEGREE)configj3 = SERVO_MAX_DEGREE;
            }
            else if(rx2_buffer[0]=='C')//Encender la camara, el brazo y el pololu
            {
                gpio_set_level(POLOLUSWITCH, 1);
            }
            else if(rx2_buffer[0]=='D')//Apagar la camara, el brazo y el pololu
            {
                gpio_set_level(POLOLUSWITCH, 0);
            }
            else if(rx2_buffer[0]=='E')//Recibir las coordenadas para modificar la configuracion del brazo
            {
                coordX = rx2_buffer[1];
                coordY = rx2_buffer[2];
                updateCoords = 1;
            }
            else if(rx2_buffer[0]=='F')
            {
                //Convertir el int a un arreglo de bytes para enviarlo.
                batlevel = adc_raw[0][0];
                uint8_t buffer[sizeof(int)];
                memcpy(buffer, &batlevel, sizeof(int));

                int bytes_sent = send(sock, buffer, sizeof(int), 0);

                if (bytes_sent < 0) {
                    ESP_LOGE(TAG, "Error al enviar datos");
                } else {
                    ESP_LOGI(TAG, "Enviados %d bytes", bytes_sent);
                }
            }
            else if(rx2_buffer[0] == 'G')
            {
                uint32_t concatR = 0;
                uint32_t concatL = 0;

                for(int a=0; a<sizeof(float); a++)
                {
                    concatL |= (uint32_t)rx2_buffer[a+1]<<(a*8);
                    concatR |=  (uint32_t)rx2_buffer[a+5]<<(a*8);
                }

                memcpy(&phi_ell, &concatL, sizeof(float));
                memcpy(&phi_r, &concatR, sizeof(float));
            }
            
            else if(rx2_buffer[0] == 'H')
            {
                uint8_t odoBuffer[sizeof(float)*3];
                odoX.input_flotante = odo_x;
                odoY.input_flotante = odo_y;
                odoTheta.input_flotante = odo_theta;

                for(int c=0; c<sizeof(float); c++)
                {
                    odoBuffer[c] = odoX.bytes[c];
                    odoBuffer[c+4] = odoY.bytes[c];
                    odoBuffer[c+8] = odoTheta.bytes[c];
                }

                int bytes_sent = send(sock, odoBuffer, sizeof(int)*3, 0);

                if (bytes_sent < 0) {
                    ESP_LOGE(TAG, "Error al enviar datos");
                } else {
                    ESP_LOGI(TAG, "Enviados %d bytes", bytes_sent);
                }

            }
        }
    }
    close(client_socket);
    vTaskDelete(NULL);
}

/*!
 \fn static void tcp_socket_init(void *arg)
 @brief Funcion para crear el socket TCP y asignar el handler.

 Inicializa el socket, colocandole una direccion y numero de puerto, manejando los errores en el proceso,
 ademas, configura al puerto para permitir multiples reconexiones. Acepta las conexiones entrantes y 
 las despliega en el monitor serial, con la direccion asignada y el puerto utilizado, tambien registra
 los eventos por cada uno de los clientes. Y por ultimo crea la tarea para manejar los distintos 
 que puede recibir al tener un cliente conectado.

 \note Al crear la tarea, si se tienen problemas de conexion o manejo de mensajes, se puede aumentar
    la prioridad de la tarea.
*/

static void tcp_socket_init(void *arg) 
{
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    //memset(&dest_addr,0,sizeof(dest_addr));

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    if(!LisSock)
    {
        LisSock = socket(addr_family, SOCK_STREAM, ip_protocol);

        if (LisSock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            return;
        }
        int opt = 1;
        setsockopt(LisSock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        ESP_LOGI(TAG, "Socket created");
    }

    int err = bind(LisSock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        close(LisSock);
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);
    //fcntl(LisSock,F_SETFL,O_NONBLOCK); //Para ser no bloqueante

    err = listen(LisSock, 1);

    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        close(LisSock);
    }

    ESP_LOGI(TAG,"Socket Listening");    

    while(1)
    {
        char addr_str[128];
        struct sockaddr_in source_addr;
        uint addr_len = sizeof(source_addr);

        sock = accept(LisSock, (struct sockaddr *)&source_addr, &addr_len);

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
        } 
        else 
        {   
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
            ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
            xTaskCreate(handle_socket,"Connection",4096,(void *)sock,configMAX_PRIORITIES-5,NULL);
            vTaskDelay(10/ portTICK_PERIOD_MS);
        }
        
    }

}

/*!
 \fn void my_post_cb(spi_slave_transaction_t *trans) 
 @brief Funcion para finalizar el envio de datos por SPI.

 Callback activado al terminar la transaccion por SPI, colocando en nivel bajo el
 pin para el handshake, lo que indica a la OpenMV Cam H7 el fin de la comunicacion.

 \param trans recibe el callback que indica cuando realizar la accion.
*/

void my_post_cb(spi_slave_transaction_t *trans) 
{
    gpio_set_level(SPI_HAND, 0);
}

/*!
 \fn void my_pre_cb(spi_slave_transaction_t *trans) 
 @brief Funcion para comenzar el envio de datos por SPI.

 Callback activado al iniciar la transaccion por SPI, colocando en nivel alto el
 pin para el handshake, indicandole a la OpenMV Cam H7 el inicio de la comunicacion y 
 que se esta listo para recibir la informacion cuando este disponible.

 \param trans recibe el callback que indica cuando realizar la accion.
*/

void my_pre_cb(spi_slave_transaction_t *trans) 
{
    gpio_set_level(SPI_HAND, 1);
}

/*!
 \fn void SPI_init_config(void)
 @brief Configura el modulo SPI, inicializa el handshake y configura el pin para el handshake.

 Inicializa la configuracion del bus SPI, asignando los pines para MISO, MOSI, CLK y el tamaño
 maximo de datos a transmitir por transaccion. Configura el bus como esclavo, asignando el pin de CS, 
 el tamaño de la cola y los callbacks al detectar una transmision. Al configurar el bus, lo inicializa
 y activa el DMA en la transaccion. Por ultimo, configura el pin de handshake como salida y establece
 el nivel logico de espera.

 \param SPI_MISO pin de MISO.
 \param SPI_MOSI pin de MOSI.
 \param SPI_CLK pin de CLK.
 \param SPI_CS pin de CS.
 \param SPI_HAND pin para realizar el handshake.

 \note El tamaño maximo de mensaje por transaccion debe de ser mayor al numero de datos que se espera recibir.
        El pin de handshake y CS puede ser cualquier pin fisico, los pines de MOSI, MISO y CLK si deben de
        coincidir con los presentados en la disposiicon de pines para el TinyS3.
*/

void SPI_init_config(void)
{
    //Inicializando la configuracion del SPI
    spi_bus_config_t SpiConfig = 
    {
        .miso_io_num = SPI_MISO,
        .mosi_io_num = SPI_MOSI,
        .sclk_io_num = SPI_CLK,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = 5120,
    };

    spi_slave_interface_config_t slavecfg =
    {
        .mode = 0,
        .spics_io_num = SPI_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_cb,
        .post_trans_cb = my_pre_cb
    };

    ESP_ERROR_CHECK(spi_slave_initialize(SPI2_HOST,&SpiConfig,&slavecfg,SPI_DMA_CH_AUTO));

    //Configurando el pin del handshake para solicitar envio de datos y recepcion
    gpio_config_t io_hand_conf={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<SPI_HAND)
    };

    gpio_config(&io_hand_conf);
    gpio_set_level(SPI_HAND, 1);

}

/*!
 \fn static void AskForPicture(void *arg)
 @brief Bucle que solicita y recibe la imagen a traves del bus SPI.

 Inicia bajo demanda la transmision de datos por el bus SPI, ante la bandera SPIAsk. En el inicio de cada
 transaccion configura el bus de recepcion y la estuctura de transaccion, para recibir el dato, luego 
 establece la longitud de la transaccion en bits e inicia la transmision. Al terminar la transaccion, se
 activa la bandera para el envio de datos por TCP, indicando que hay datos disponibles y baja la bandera de
 SPIAsk, ya que se cumplio con la solicitud de imagen.

 \note se utiliza spi_slave_transmit, ya que este inicia la transaccion y es capaz de determinar la cantidad
 de datos recibidos, si son necesarios. Al no configurar el buffer de TX, solo se esperan los datos.

*/

static void AskForPicture(void *arg)
{   
    while(1){
    if(SPIAsk)
    {
    spi_slave_transaction_t spi_slave;
    memset(rx_buffer,0x30,BUFFER_SIZE);
    memset(&spi_slave,0,sizeof(spi_slave));

    spi_slave.length = BUFFER_SIZE*8;
    spi_slave.rx_buffer = rx_buffer;

    spi_slave_transmit(SPI2_HOST,&spi_slave,portMAX_DELAY);
    //spi_slave_queue_trans(SPI2_HOST,&spi_slave,portMAX_DELAY);
    //spi_slave_get_trans_result(SPI2_HOST,&spi_dummy,portMAX_DELAY);
    SPIAsk = 0;
    tcpavail = 1;
    }

    vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}

/*!
 \fn static void mcpwm_initialize(void)
 @brief Configura los 3 canales de PWM utilizados para el manipulador serial.

 Inicializa los pines GPIO para los canales PWM, utilizando mcpwm_gpio_init. Luego, configura los parametros
 de PWM como la frecuencia en Hz de operacion, el ciclo de trabajo inicial y el modo de reloj para su 
 operacion. Se inicializan los temporizadores 0, 1 y 2 del modulo MCPWM y se configuran con los parametros
 indicados.

 \note Se recomeinda dejar un retraso de 1 segundo luego de llamar esta funcion, esto evita que los
 servomotores se muevan de manera abrupta en el inicio de la configuracion.

*/

static void mcpwm_initialize(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_1); // Set GPIO 1 as the PWM0A, which is the output for MCPWM0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_NUM_2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_NUM_3);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;                       // Set frequency in Hz (50Hz for most servos)
    pwm_config.cmpr_a = 2000;                           // Initial duty cycle
    pwm_config.counter_mode = MCPWM_UP_COUNTER;       // Up counter mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;         // Active high PWM duty mode
    
    mcpwm_config_t pwm_config2;
    pwm_config2.frequency = 50;                       // Set frequency in Hz (50Hz for most servos)
    pwm_config2.cmpr_a = 2000;                           // Initial duty cycle
    pwm_config2.counter_mode = MCPWM_UP_COUNTER;       // Up counter mode
    pwm_config2.duty_mode = MCPWM_DUTY_MODE_0;         // Active high PWM duty mode

    mcpwm_config_t pwm_config3;
    pwm_config3.frequency = 50;                       // Set frequency in Hz (50Hz for most servos)
    pwm_config3.cmpr_a = 2178;                           // Initial duty cycle
    pwm_config3.counter_mode = MCPWM_UP_COUNTER;       // Up counter mode
    pwm_config3.duty_mode = MCPWM_DUTY_MODE_0;         // Active high PWM duty mode

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config2);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config3);
}

/*!
 \fn static void updatePositionSCH(void *arg)
 @brief Controla la actualizacion y el ajuste continuo de las posiciones de los servomotores.

 Limita los valores de configuracion de cada junta, para evitar colisiones fisicas. Luego, inicializa
 las variables de paso para cada uno de los servomotores con su valor actual y los ajusta gradualmente
 hasta coincidir con la nueva configuracion, en paralelo ajusta los ciclos de trabajo para que mover 
 fisicamente el brazo. Este ajusta se realiza por pasos y los haces a pequeños intervalos, lo que 
 asegura un movimiento suave en las juntas.

 \note Luego de inicializar el MCPWM, se debe de asignar la configuracion de reposo a las juntas 
 por codigo, para que se actualice la posicion de manera suave.

*/

static void updatePositionSCH(void *arg)
{
    while(1)
    {
        if(configj1 > 110)configj1 = 110;
        if(configj2 > SERVO_MAX_DEGREE)configj2 = SERVO_MAX_DEGREE;
        else if(configj2 < 45)configj2 = 45;
        if(configj3 > SERVO_MAX_DEGREE)configj3 = SERVO_MAX_DEGREE;
        else if(configj3 < 40)configj3 = 40;

        step1 = brazo.joint1;
        step2 = brazo.joint2;
        step3 = brazo.gripper;

        while((brazo.joint1 != configj1) | (brazo.joint2 != configj2) | (brazo.gripper != configj3))
        {
            brazo.joint1 = step1;
            brazo.joint2 = step2;
            brazo.gripper = step3;

            if(step1>configj1)
            {
                brazo.duty_us1 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step1) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, brazo.duty_us1);
                step1--;
                
            }
            else if(step1<configj1)
            {
                brazo.duty_us1 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step1) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, brazo.duty_us1);
                step1++; 
                
            }

            if(step2>configj2)
            {
                brazo.duty_us2 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step2) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, brazo.duty_us2);
                step2--;
                
            }
            else if(step2<configj2)
            {
                brazo.duty_us2 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step2) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, brazo.duty_us2);
                step2++; 
                
            }

            if(step3>configj3)
            {
                brazo.duty_us3 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step3) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, brazo.duty_us3);
                step3--;
                
            }
            else if(step3<configj3)
            {
                brazo.duty_us3 = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * step3) / SERVO_MAX_DEGREE));
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, brazo.duty_us3);
                step3++; 
                
            }

            vTaskDelay(15/ portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*!
 \fn static void CalcConfig(void *arg)
 @brief Calcula la configuracion XY del manipulador serial.

 Calcula las posiciones angulares optimas para los servomotores del manipulador serial, utilizando
 trigonometria y las coordenadas deseadas para mover el efector final. Luego, ajusta los valores 
 calculados para que no pasen los limites establecidos y coloca estos valores calculados como los 
 nuevos valores deseados, para que sean actualizados.

*/

static void CalcConfig(void *arg)
{
    while(1)
    {
       if(updateCoords > 0)
       {
        double Yadjust = (double)coordY;

        double tempq2 = -acos(((double)(coordX*coordX)+(double)(Yadjust*Yadjust)-(double)(es1*es1)-(double)(es2*es2))/((double)(2*es1*es2)));
        
        double tempq1 = atan((double)Yadjust/(double)coordX) - atan(((double)es2*sin(tempq2))/((double)es1+(double)es2*cos(tempq2)));

        q1 = (uint32_t)round(tempq1*(180.0/M_PI));
        q2 = (uint32_t)round(tempq2*(180.0/M_PI) + 180.0);

        if(q1 > 110) q1 = 110;
        
        if(q2 > 180) q2 = 180;
        else if(q2 < 45) q2 = 45;

        configj1 = q1;
        configj2 = q2;

        updateCoords = 0;
       }
       vTaskDelay(10/ portTICK_PERIOD_MS); 
    }
}

/*
uint8_t* encode_cbor(const char* data, size_t* mesSize)
{
    uint8_t* CBORBuffer = (uint8_t*)malloc(BUFFER_SIZE+10);

    CborEncoder encoder;

    cbor_encoder_init(&encoder,CBORBuffer,(BUFFER_SIZE+10),0);

    cbor_encode_byte_string(&encoder, (const uint8_t*)data,(BUFFER_SIZE+10));

    *mesSize = cbor_encoder_get_buffer_size(&encoder,CBORBuffer);

    return CBORBuffer;
}
*/

/*
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
*/

/*!
 \fn void ADCConfig(void)
 @brief Configura e inicializa un canal ADC.

 Inicializa una nueva unidad ADC usando la configuracion de un solo disparo. Luego, configura el 
 canal ADC especificado con un ancho de bits predeterminado y un atenuador de 11 dB, para leer 
 un rango entre 0 a 3V. Por ultimo, se calibra el ADC para las medidas.

 \note Se puede modificar la atenuacion dependiendo del nivel analogico a leer.

*/

void ADCConfig(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_4, &config));

    //adc_cali_handle_t adc_cali_chan4_handle = NULL;
    //bool calibration_chan4 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_4, ADC_ATTEN_DB_0, &adc_cali_chan4_handle);

}

/*!
 \fn static void BatteryUpadte(void *arg)
 @brief Realiza lecturas periodicas del nivel de bateria utilizando el ADC.

 Realiza una lectura del canal ADC cada segundo, obteniendo los valores en bruto del
 nivel de bateria.

*/

static void BatteryUpadte(void *arg)
{
    while(1)
    {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_raw[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, ADC_CHANNEL_4, adc_raw[0][0]);
        vTaskDelay(1000/ portTICK_PERIOD_MS); 
    }
}

/*!
 \fn static void uart_tx_task(void * p_params)
 @brief Transmite periodicamente datos a traves del UART.

 Configura el tiempo de control y establece la frecuencia de tranmsision para el UART,
 luego se rastrea el tiempo para garantizar de manera regular el envio de datos. En cada
 ciclo se codifica las velocidades de la rueda con CBOR y luego se envia por medio de UART
 al terminar la codificacion.

 \note Los datos pueden ser codificados con CBOR o se puede utilizar para enviar
 otra estructura de datos.

*/

static void uart_tx_task(void * p_params)
{
    TickType_t last_control_time;
    const TickType_t control_freq_ticks = pdMS_TO_TICKS(control_time_ms);
    size_t numbytes;

    // Initialise the xLastWakeTime variable with the current time
    last_control_time = xTaskGetTickCount();

    while(1)
    {
        // Wait for the next cycle
        vTaskDelayUntil(&last_control_time, control_freq_ticks);
        numbytes = cbor_encode_wheel_speeds();
        uart_write_bytes(UART_PORT , cbor_buf_tx, numbytes);       
    }
}

bool cbor_parse_odometry_fsm(void)
{
    bool b_is_invalid_data = false; 
    int ty;
    int idx = 0;
    
    cbor_parser_init(cbor_buf_rx, sizeof(cbor_buf_rx), 0, &parser, &it);

    while((!cbor_value_at_end(&it)) && (!b_is_invalid_data))
    {
        ty = cbor_value_get_type(&it);
        
        switch(parser_state)
        {
            case PARSER_START:
            {
                if(ty == CborArrayType)
                {
                    cbor_value_enter_container(&it, &arr);
                    idx = 0;
                    parser_state = PARSER_IS_INSIDE_ARRAY;
                }
                else
                {
                    b_is_invalid_data = true;
                }
                break;
            }
            case PARSER_IS_INSIDE_ARRAY:
            case PARSER_GOT_X_POS:
            case PARSER_GOT_Y_POS:
            {
                if(ty == CborFloatType)
                {
                    cbor_value_get_float(&arr, &xi_odo[idx++]);
                    cbor_value_advance(&arr);
                    parser_state++;
                    //printf("%f\n", xi_odo[idx-1]);
                    //ESP_LOGI(TAG, "%f\n", xi_odo[idx-1]);
                }
                else
                {
                    b_is_invalid_data = true;
                    parser_state = PARSER_START;
                    ESP_LOGI(TAG, "Error parsing CBOR data");
                }
                break;
            }
            case PARSER_GOT_ANGLE:
            {
                if(ty == CborInvalidType)
                {
                    if(cbor_value_at_end(&arr))
                    {
                        cbor_value_leave_container(&it, &arr);
                        parser_state = PARSER_START;
                        break;
                    }
                }
                b_is_invalid_data = true;
                parser_state = PARSER_START;
                break;
            }
            default:
                b_is_invalid_data = true;
                parser_state = PARSER_START;
                break;
        }
    }

    return b_is_invalid_data;
}


void decodeCBOR(uint8_t* buffer, size_t len) {
    CborParser parser;
    CborValue value;
    cbor_parser_init(buffer, len, 0, &parser, &value); // Initialize the parser

    if (!cbor_value_is_container(&value)) {
        printf("CBOR value is not a container.\n");
        return;
    }

    CborValue array;
    cbor_value_enter_container(&value, &array); // Enter the array

    float decoded_values[3];
    for (size_t i = 0; i < 3; i++) {
        if (!cbor_value_is_float(&array)) {
            printf("CBOR value is not a float.\n");
            return;
        }
        cbor_value_get_float(&array, &decoded_values[i]); // Extract the float value
        cbor_value_advance_fixed(&array); // Move to the next value in the array
    }

    // Now you can use decoded_values array for your logic
    odo_x = decoded_values[0];
    odo_y = decoded_values[1];
    odo_theta = decoded_values[2];
}

void 
uart_rx_task(void * p_params)
{
    int rx_bytes;

    while (1) 
    {
        rx_bytes = uart_read_bytes(UART_PORT, cbor_buf_rx, sizeof(cbor_buf_rx), odoread_time_ms / portTICK_PERIOD_MS);
        
        if (rx_bytes >= CBOR_MSG_MIN_SIZE) 
        {
            decodeCBOR(cbor_buf_rx,rx_bytes);
            uart_flush(UART_PORT);
        }
    }
}

void app_main(void)
{   

    //Inicializando el UART
    UART_init();

    //Configurando el pin para encender el Pololu3pi+
    gpio_config_t pololuPin={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<POLOLUSWITCH)
    };

    gpio_config(&pololuPin);
    
    //Configurando el MCPWM para manejar los servos y colocandolos en reposo
    mcpwm_initialize();
    configj1 = 90;
    configj2 = 90;
    configj3 = 160;
    vTaskDelay(1000 / portTICK_PERIOD_MS); //Seguridad, evita que los servomotores salten.
    
    //Configuracion del ADC
    ADCConfig();

    //Configurando el SPI
    SPI_init_config();
    //Inicializando el NVS
    nvs_init();
    //Inicializando el WIFI
    wifi_init_softap();

    // xTaskCreate(tcp_socket_init,"TCPSocket",4096,NULL,configMAX_PRIORITIES-2,NULL);
    // xTaskCreate(AskForPicture,"SPIRetrieve",4096,NULL,configMAX_PRIORITIES-2,NULL);
    // xTaskCreate(TCPSendRobust,"TCPSend",4096,NULL,configMAX_PRIORITIES-1,NULL);
    // xTaskCreate(updatePositionSCH,"MOVServo",1024,NULL,configMAX_PRIORITIES-1,NULL);
    // xTaskCreate(CalcConfig,"CalcPosition",1024,NULL,configMAX_PRIORITIES-4,NULL);
    // xTaskCreate(BatteryUpadte,"UpdateBateria",4096,NULL,configMAX_PRIORITIES-5,NULL);
    // xTaskCreate(uart_tx_task, "uart_tx_task", 4096, NULL, configMAX_PRIORITIES-5, NULL);

    // Ejemplo de asignación de tareas a núcleos específicos
    xTaskCreatePinnedToCore(tcp_socket_init, "TCPSocket", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0); // Tarea en el núcleo 0
    xTaskCreatePinnedToCore(AskForPicture, "SPIRetrieve", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0); // Tarea en el núcleo 0
    //xTaskCreatePinnedToCore(TCPSendRobust, "TCPSend", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0); // Tarea en el núcleo 1
    xTaskCreate(TCPSendRobust,"TCPSend",4096,NULL,configMAX_PRIORITIES-1,NULL);// Dejar que se coloque en el mejor nucleo (Funciona mejor)
    xTaskCreatePinnedToCore(updatePositionSCH, "MOVServo", 1024, NULL, configMAX_PRIORITIES - 1, NULL, 1); // Tarea en el núcleo 1
    xTaskCreatePinnedToCore(CalcConfig, "CalcPosition", 1024, NULL, configMAX_PRIORITIES - 4, NULL, 1); // Tarea en el núcleo 0
    xTaskCreatePinnedToCore(BatteryUpadte, "UpdateBateria", 4096, NULL, configMAX_PRIORITIES - 5, NULL, 1); // Tarea en el núcleo 1
    xTaskCreatePinnedToCore(uart_tx_task, "uart_tx_task", 4096, NULL, configMAX_PRIORITIES - 5, NULL, 0); // Tarea en el núcleo 0
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, configMAX_PRIORITIES - 5, NULL, 0); // Tarea en el núcleo 0
}