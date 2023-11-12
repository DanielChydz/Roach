#include <Config.hpp>
#include <Credentials.hpp>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
#include <MessageProcessor.hpp>
#include <Controller.hpp>
#include <cstring> // strcpy
#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

bool disconnectAction = false;
bool connected = false;
bool dots = false;

TaskHandle_t xDotsHandle;
TaskHandle_t xUDPHandle;

void udp_server_task(void *pvParameters);
void udp_client_task(void *pvParameters);

// task, I think the name is self explanatory
void printDotsWhileConnectingToWifi(void *param){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, 300); // wait for debug messages to pass
    printf("Oczekiwanie na polaczenie");
    while(!connected){
        printf(".");
        vTaskDelayUntil(&xLastWakeTime, 100);
    }
}

static void wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("Wi-Fi", "Polaczono z siecia wifi.");
            disconnectAction = false;
            connected = true;
            vTaskDelete(xDotsHandle);

            xTaskCreatePinnedToCore(
                udp_server_task,      // Function that should be called
                "UDPServer",    // Name of the task (for debugging)
                10000,               // Stack size (bytes)
                NULL,               // Parameter to pass
                UDPClientPriority,                  // Task priority
                &xUDPHandle,               // Task handle
                UDPClientCore          // Core you want to run the task on (0 or 1)
            );
            xTaskCreatePinnedToCore(
                udp_client_task,      // Function that should be called
                "UDPClient",    // Name of the task (for debugging)
                10000,               // Stack size (bytes)
                NULL,               // Parameter to pass
                UDPClientPriority,                  // Task priority
                &xUDPHandle,               // Task handle
                UDPClientCore          // Core you want to run the task on (0 or 1)
            );
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if(!disconnectAction){
                ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Zatrzymywanie systemu.");
                if(connected) vTaskDelete(xUDPHandle); // prevent from deleting null task
                //brake();
                xTaskCreatePinnedToCore(
                    printDotsWhileConnectingToWifi,      // Function that should be called
                    "printDotsWhileConnectingToWifi",    // Name of the task (for debugging)
                    10000,               // Stack size (bytes)
                    NULL,               // Parameter to pass
                    printDotsWhileConnectingToWifiPriority,                  // Task priority
                    &xDotsHandle,               // Task handle
                    printDotsWhileConnectingToWifiCore          // Core you want to run the task on (0 or 1)
                );
                connected = false;
                disconnectAction = true;
            }
            esp_wifi_connect();
            break;
    }
}

void connectWifi(){
    nvs_flash_init(); // saving wifi data between power cycles, for some reason it's necessary
    esp_netif_init(); // initialising TCP/IP layer
    esp_event_loop_create_default();     // creating event loop for handling events
    esp_netif_create_default_wifi_sta(); // creating wifi station with default settings
    wifi_init_config_t wifiInitiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifiInitiation);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifiEventHandler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifiEventHandler, NULL);
    wifi_config_t wifiConfiguration = {
        .sta = {
            .ssid = SSID,
            .password = PASSWORD,
        }
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifiConfiguration);
    esp_wifi_start();
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_connect();

    // It's a weird way to deal with reconnection, especially given the events that can do the same thing.
    // The problem is there's an issue causing the wifi to stop connecting after exiting this function (deleting while loop) and attempting to connect in event handler above.
    // Since the official way (available in documentation) doesn't work, it's necessary to stick to this workaround.
    // https://www.esp32.com/viewtopic.php?t=25230
    while(true){
        vTaskDelay(100);
        if(!connected){
            ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Restartowanie modulu wifi i ponowne laczenie.");
            esp_wifi_stop();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_start();
            esp_wifi_connect();
            while(!connected)vTaskDelay(20);
        }
    }
}


// task for handling UDP client socket
void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    if (sock < 0) {
        ESP_LOGE("UDPS", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(4321);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("UDPS", "Socket unable to bind: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI("UDPS", "Waiting for data");
        socklen_t addr_len = sizeof(client_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);

        if (len < 0) {
            ESP_LOGE("UDPS", "recvfrom failed: errno %d", errno);
            break;
        } else {
            rx_buffer[len] = 0;
            ESP_LOGI("UDPS", "Received %d bytes from %s:", len, inet_ntoa(client_addr.sin_addr));
            ESP_LOGI("UDPS", "%s", rx_buffer);
        }
    }

    if (sock != -1) {
        ESP_LOGE("UDPS", "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

void udp_client_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr("192.168.220.201");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(4322);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE("UDPC", "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        const char *message = "Hello, UDP server!";
        int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE("UDPC", "Error occurred during sending: errno %d", errno);
        } else {
            ESP_LOGI("UDPC", "Message sent to server");
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Send message every 2 seconds
    }

    if (sock != -1) {
        ESP_LOGE("UDPC", "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}


void udpPrepareMessage(int subject){
    char messageBuffer[256];
    string messageBufferString;

    messageBufferString.append("1");
    messageBufferString.append(to_string(123).substr(0, 5));
    messageBufferString.append("2");
    strcpy(messageBuffer, messageBufferString.c_str());
    // udpSend(messageBuffer);
    messageBufferString = "";
}