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
//Manejo de los LEDC
#include "driver/ledc.h"
//Relacionado al SPI
#include "driver/spi_slave.h"
#include "driver/spi_master.h"

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
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Resolucion del ciclo de trabajo de 13 bits

//Parametros para configurar el canal y pin del servo con el LEDC
#define SERVO1_CHANNEL          LEDC_CHANNEL_0
#define SERVO1_PIN              GPIO_NUM_1
#define SERVO2_CHANNEL          LEDC_CHANNEL_1
#define SERVO2_PIN              GPIO_NUM_2
#define SERVO3_CHANNEL          LEDC_CHANNEL_2
#define SERVO3_PIN              GPIO_NUM_3

//Parametros para calcular el duty en base al angulo
const float minDeg = 0.0;
const float maxDeg = 180.0;
const float minDuty = 204.0;
const float maxDuty = 1024.0;
float dutyValue = 0;

//Configuracion de pines para el SPI
#define SPI_NAME    SPI2_HOST
#define SPI_MISO    GPIO_NUM_37
#define SPI_MOSI    GPIO_NUM_35
#define SPI_CLK     GPIO_NUM_36
#define SPI_CS      GPIO_NUM_34
#define SPI_HAND    GPIO_NUM_9

//Variables para las solicitudes
#define BUFFER_SIZE 1600 //antes 1024
char rx_buffer[BUFFER_SIZE];
char recvbuf[3];
uint8_t SPIAsk = 0;
spi_device_interface_config_t CamH7;
spi_device_handle_t  SPIHandle;

//TAG para el log de errores o informacion
static const char *TAG = "Prueba";

//Prototipos para algunas funciones
static void AskForPicture(void *arg);
static void SetAngle(int channel, int angle);

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
            int to_write = BUFFER_SIZE;

            while (to_write > 0) {
                int written = send(sock, &rx_buffer, to_write, 0);
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
        char rx2_buffer[4];
        len = recv(client_socket, rx2_buffer, sizeof(rx2_buffer)-1, 0);

        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
            break;

        } else if (len == 0) {
            ESP_LOGE(TAG, "Connection closed");
            break;

        } else {
            ESP_LOGI(TAG, "Received %d bytes", len);
            char pic_send = 'A';
            // if(strchr(rx2_buffer,pic_send) != NULL)
            // {   
            //     SPIAsk = 1;
            // }
            if(rx2_buffer[0]=='A')
            {
                SPIAsk = 1;
            }
            else if(rx2_buffer[0] == 'B')
            {
                SetAngle(SERVO1_CHANNEL,rx2_buffer[1]);
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
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50, // (1/20 mS) O 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t servo_channel[3];
    //Configuracion del canal y pin del servo 1
    servo_channel[0].channel = SERVO1_CHANNEL;
    servo_channel[0].gpio_num = SERVO1_PIN;
    //Configuracion del canal y pin del servo 2
    servo_channel[1].channel = SERVO2_CHANNEL;
    servo_channel[1].gpio_num = SERVO2_PIN;
    //Configuracion del canal y pin del servo 3
    servo_channel[2].channel = SERVO3_CHANNEL;
    servo_channel[2].gpio_num = SERVO3_PIN;
    //Configuracion de los parametros generales entre canales
    for (size_t i = 0; i < 3; i++)
    {
        servo_channel[i].speed_mode =   LEDC_MODE;
        servo_channel[i].timer_sel =    LEDC_TIMER;
        servo_channel[i].intr_type =    LEDC_INTR_DISABLE;
        servo_channel[i].duty =         615; //5% de duty o posicion 0 grados del servo
        servo_channel[i].hpoint =       0;

        ESP_ERROR_CHECK(ledc_channel_config(&servo_channel[i]));
    }

}

static void SetAngle(int channel, int angle)
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
        .max_transfer_sz = 4800,
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

void app_main(void)
{   

    //Configurando el SPI
    SPI_init_config();
    //Inicializando el NVS
    nvs_init();
    //Inicializando el WIFI
    wifi_init_softap();
    
     //Actualizacion del valor del PWM (angulo del servo)
    servo_ledc_config();
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // SetAngle(SERVO1_CHANNEL,0);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // SetAngle(SERVO1_CHANNEL,90);
    // vTaskDelay(pdMS_TO_TICKS(2000));
    // SetAngle(SERVO1_CHANNEL,180);
    
    xTaskCreate(tcp_socket_init,"TCPSocket",4096,NULL,configMAX_PRIORITIES-3,NULL);
    xTaskCreate(AskForPicture,"SPIRetrieve",4096,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(TCPSendRobust,"TCPSend",4096,NULL,configMAX_PRIORITIES-1,NULL);
    
}