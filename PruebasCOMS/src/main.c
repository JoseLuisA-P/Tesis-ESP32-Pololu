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

// Setup UART buffered IO with event queue
static const int uart_buffer_size = 1024*5;

// Puertos y conexiones a utilizar
//TX y RX asignados libremente
#define TXD_PIN             (GPIO_NUM_4)
#define RXD_PIN             (GPIO_NUM_5) 
#define uart_num            UART_NUM_2
int num = 0;
char data[1024*5+1];
uint8_t avail = 0;

//Parametros para la configuracion del servidor
#define SSID                "ESP32-AP"
#define WIFIPASSWORD        "1234567890"
#define WIFICHANNEL         2
#define MAXCONNECTIONS      5

//Puerto para la comunicacion
#define PORT 3333

//TAG para el log de errores o informacion
static const char *TAG = "Prueba";

void uart_init_config(void)
{   
    const uart_config_t uart_config = {
        .baud_rate = 921600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .source_clk = UART_SCLK_APB,
    };
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 0, NULL, 0));
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); 
    // Set UART pins(TX: IO17, RX: IO16, NOT USED, NOT USED)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); 
    
}

static void tx_task(void *arg)
{

    while (1) {	

        if(avail > 0){
            uart_write_bytes(uart_num, &data, uart_buffer_size);
            //printf("Enviado\n");
            avail = 0;
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    while(1)
    {
        const int tam = uart_read_bytes(uart_num,data,uart_buffer_size, 100/ portTICK_PERIOD_MS);
        if (tam > 0) {
            printf ("Recibido\n");
            avail = 1;
        }
    }
    
}

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

static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation. 
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

static void tcp_server_init(void *arg)
{
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr_ip4->sin_family = AF_INET;
    dest_addr_ip4->sin_port = htons(PORT);
    ip_protocol = IPPROTO_IP;

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);

    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while(1)
    {
        ESP_LOGI(TAG,"Socket Listening");

        struct sockaddr_in6 source_addr;
        uint addr_len = sizeof(source_addr);

        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);   
        
        do_retransmit(sock);
        
        shutdown(sock, 0);
        close(sock);

    }

    CLEAN_UP:
        close(listen_sock);
        vTaskDelete(NULL);

}

void app_main(void)
{   
    //Inicializando el UART
    uart_init_config();
    //Inicializando el NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    //Inicializando el WIFI
    wifi_init_softap();
    //tcp_server_init(NULL);

    xTaskCreate(rx_task, "uart_rx_task", 1024, NULL, 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024, NULL, 1, NULL);
    //Creando el socket TCP para el envio de datos
    xTaskCreate(tcp_server_init,"TCPSocket",1024*4,NULL,3,NULL);
}