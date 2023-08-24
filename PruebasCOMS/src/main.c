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

typedef struct {
    uint32_t joint1;
    uint32_t joint2;
    uint32_t gripper;
    uint32_t duty_us1;
    uint32_t duty_us2;
    uint32_t duty_us3;
}   ManSer;

ManSer brazo = {5,5,5,0,0,0};

uint32_t configj1,configj2,configj3;
uint32_t step1,step2,step3;

uint32_t q1, q2; //Configuraciones calculadas
uint32_t es1 = 47; //Largo eslabon 1
uint32_t es2 = 68; //Largo eslabon 2 
uint8_t coordX,coordY;//Configuraciones en XY para colocar el brazo
uint8_t updateCoords = 0;//Para actualizar la configuracion luego de enviar coordenadas

//Prototipos para algunas funciones
static void AskForPicture(void *arg);
static void SetAngle(int channel, uint32_t angle);
uint8_t* encode_cbor(const char* data, size_t* mesSize);

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

static void TCPSendRobust(void *arg)
{
    while(1)
    {
        if(tcpavail>0)
        {
            printf ("TCPEnviado\n");
            //size_t messSize;
            //uint8_t* codedData = encode_cbor(rx_buffer,&messSize);
            int to_write = BUFFER_SIZE;
            //int to_write = messSize;

            while (to_write > 0) {
                int written = send(sock, &rx_buffer, to_write, 0);
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

void handle_socket(void *pvParameters)
{
    int client_socket = (int)pvParameters;

    while(1) 
    {
        int len;
        char rx2_buffer[5];
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
                if(configj1 > SERVO_MAX_DEGREE)configj1 = SERVO_MAX_DEGREE;
                configj2 = rx2_buffer[2];
                if(configj2 > SERVO_MAX_DEGREE)configj2 = SERVO_MAX_DEGREE;
                configj3 = rx2_buffer[3];
                if(configj3 > SERVO_MAX_DEGREE)configj3 = SERVO_MAX_DEGREE;
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
        }
    }
    close(client_socket);
    vTaskDelete(NULL);
}

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
            xTaskCreate(handle_socket,"Connection",4096,(void *)sock,5,NULL);
            vTaskDelay(10/ portTICK_PERIOD_MS);
        }
        
    }

}

void servo_ledc_config(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50, // (1/20 mS) O 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t servo_chanel1;

    servo_chanel1.channel =      SERVO1_CHANNEL;
    servo_chanel1.gpio_num =     SERVO1_PIN;
    servo_chanel1.speed_mode =   LEDC_LOW_SPEED_MODE;
    servo_chanel1.timer_sel =    LEDC_TIMER_0;
    servo_chanel1.intr_type =    LEDC_INTR_DISABLE;
    servo_chanel1.duty =         750; //5% de duty o posicion 0 grados del servo
    servo_chanel1.hpoint =       0;

    ESP_ERROR_CHECK(ledc_channel_config(&servo_chanel1));

}

static void SetAngle(int channel, uint32_t angle)
{
    dutyValue = minDuty + ((maxDuty-minDuty)/(maxDeg-minDeg))*(angle-minDeg);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, dutyValue));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

void my_post_cb(spi_slave_transaction_t *trans) 
{
    //Utilizado para activar el pin despues de la transaccion
    gpio_set_level(SPI_HAND, 0);
}

void my_pre_cb(spi_slave_transaction_t *trans) 
{
    //Utilizado para apagar el pin antes de la transaccion
    gpio_set_level(SPI_HAND, 1);
}

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

static void mcpwm_initialize(void)
{
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_NUM_1); // Set GPIO 1 as the PWM0A, which is the output for MCPWM0
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_NUM_2);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_NUM_3);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;                       // Set frequency in Hz (50Hz for most servos)
    pwm_config.cmpr_a = 0;                           // Initial duty cycle in %, 0% for 0-degree position
    pwm_config.counter_mode = MCPWM_UP_COUNTER;       // Up counter mode
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;         // Active high PWM duty mode
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
}

static void updatePositionSCH(void *arg)
{
    while(1)
    {
        if(configj1 > SERVO_MAX_DEGREE)configj1 = SERVO_MAX_DEGREE;
        if(configj2 > SERVO_MAX_DEGREE)configj2 = SERVO_MAX_DEGREE;
        if(configj3 > SERVO_MAX_DEGREE)configj3 = SERVO_MAX_DEGREE;

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

static void CalcConfig(void *arg)
{
    while(1)
    {
       if(updateCoords > 0)
       {

        double tempq2 = -acos(((coordX^2)+(coordY^2)-(es1^2)-(es2^2))/(2*es1*es2));
        
        double tempq1 = atan(coordY/coordX) - atan((es2*sin(tempq2))/(es1+es2*cos(tempq2)));

        double deg = 180.00;

        q1 = (uint32_t)(tempq1*(deg/M_PI));
        q2 = (uint32_t)(tempq2*(deg/M_PI));

        if(q1 > 180) q1 = 180;
        
        if(q2 > 180) q2 = 180;

        configj1 = q1;
        configj2 = q2 - 90;

        updateCoords = 0;
       }
       vTaskDelay(10/ portTICK_PERIOD_MS); 
    }
}

uint8_t* encode_cbor(const char* data, size_t* mesSize)
{
    uint8_t* CBORBuffer = (uint8_t*)malloc(BUFFER_SIZE+10);

    CborEncoder encoder;

    cbor_encoder_init(&encoder,CBORBuffer,(BUFFER_SIZE+10),0);

    cbor_encode_byte_string(&encoder, (const uint8_t*)data,(BUFFER_SIZE+10));

    *mesSize = cbor_encoder_get_buffer_size(&encoder,CBORBuffer);

    return CBORBuffer;
}

void app_main(void)
{   

    //Configurando el pin para encender el Pololu3pi+
    gpio_config_t pololuPin={
        .intr_type=GPIO_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pin_bit_mask=(1<<POLOLUSWITCH)
    };

    gpio_config(&pololuPin);
    //gpio_set_level(POLOLUSWITCH, 0);
    
    //Configurando el MCPWM para manejar los servos y colocandolos en 0
    mcpwm_initialize();
    //updatePosition(0,0,0);
    configj1 = 0;
    configj2 = 90;
    configj3 = 160;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    //Configurando el SPI
    SPI_init_config();
    //Inicializando el NVS
    nvs_init();
    //Inicializando el WIFI
    wifi_init_softap();
    
    xTaskCreate(tcp_socket_init,"TCPSocket",4096,NULL,configMAX_PRIORITIES-3,NULL);
    xTaskCreate(AskForPicture,"SPIRetrieve",4096,NULL,configMAX_PRIORITIES-2,NULL);
    xTaskCreate(TCPSendRobust,"TCPSend",4096,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(updatePositionSCH,"MOVServo",1024,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(CalcConfig,"CalcPosition",1024,NULL,configMAX_PRIORITIES-4,NULL);
}