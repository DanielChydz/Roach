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
bool firstBoot = true;

TaskHandle_t xDotsHandle;
TaskHandle_t xUDPServerHandle;
TaskHandle_t xUDPClientHandle;
TaskHandle_t xWifiServiceHandle;

void udp_server_task(void *pvParameters);
void udp_client_task(void *pvParameters);
void wifiServiceTask(void *pvParameters);

// task printing dots to the console when waiting for the wifi to connect
void printDotsWhileConnectingToWifi(void *param){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, 300); // wait for debug messages to pass
    printf("Oczekiwanie na polaczenie z wifi");
    while(!connected){
        printf(".");
        vTaskDelayUntil(&xLastWakeTime, printDotsWhileConnectingToWifiDelay / portTICK_PERIOD_MS);
    }
}

static void wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("Wi-Fi", "Polaczono z siecia wifi.");
            disconnectAction = false;
            vTaskDelete(xDotsHandle);
            break;
        case WIFI_EVENT_WIFI_READY:
            ESP_LOGI("Wi-Fi", "Wifi gotowe do dzialania.");
            connected = true;
            if(!firstBoot){
                vTaskResume(xUDPClientHandle);
                vTaskResume(xUDPServerHandle);
            }
            if(firstBoot) firstBoot = false;
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if(!disconnectAction){
                ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Zatrzymywanie systemu.");
                //brake();
                if(connected){
                    vTaskSuspend(xUDPClientHandle);
                    vTaskSuspend(xUDPServerHandle);
                }
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

// start wifi and udp tasks
void startWifiService(){
    xTaskCreatePinnedToCore(
        wifiServiceTask,      // Function that should be called
        "connectWifiTask",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        wifiServicePriority,                  // Task priority
        &xWifiServiceHandle,               // Task handle
        wifiServiceCore          // Core you want to run the task on (0 or 1)
    );
    while(!connected) vTaskDelay(10);
    xTaskCreatePinnedToCore(
        udp_server_task,      // Function that should be called
        "UDPServer",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        udpServerPriority,                  // Task priority
        &xUDPServerHandle,               // Task handle
        udpServerCore          // Core you want to run the task on (0 or 1)
    );
    xTaskCreatePinnedToCore(
        udp_client_task,      // Function that should be called
        "UDPClient",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        udpClientPriority,                  // Task priority
        &xUDPClientHandle,               // Task handle
        udpClientCore          // Core you want to run the task on (0 or 1)
    );
}

// task maintaining wifi connection
void wifiServiceTask(void *pvParameters){
    TickType_t xLastWakeTime = xTaskGetTickCount();
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
    // The problem is there's an issue causing the wifi to stop connecting after exiting wifiServiceTask function (deleting while loop) and attempting to connect in event handler above.
    // Since the official way (available in documentation) doesn't work, it's necessary to stick to this workaround.
    // https://www.esp32.com/viewtopic.php?t=25230
    while(true){
        if(!connected){
            ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Restartowanie modulu wifi i ponowne laczenie.");
            esp_wifi_stop();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_wifi_start();
            esp_wifi_connect();
            while(!connected) vTaskDelayUntil(&xLastWakeTime, wifiServiceWaitForConnectionDelay / portTICK_PERIOD_MS);
        }
        vTaskDelayUntil(&xLastWakeTime, wifiServiceCheckConnectionDelay / portTICK_PERIOD_MS);
    }
}

// task for handling UDP server
void udp_server_task(void *pvParameters) {
    char rx_buffer[128];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);

    if (sock < 0) {
        ESP_LOGE("UDP Server", "Nie mozna utworzyc gniazda, nr bledu: %d. Uruchom ponownie ESP32.", errno);
        vTaskDelete(NULL);
        return;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(ConnectivityData.udpReceivePort);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("UDP Server", "Nie mozna przypiąć IP i portu do gniazda, nr bledu: %d. Uruchom ponownie ESP32.", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        ESP_LOGI("UDP Server", "Oczekiwanie na pakiet danych.");
        socklen_t addr_len = sizeof(client_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&client_addr, &addr_len);

        if (len < 0) {
            ESP_LOGE("UDP Server", "Blad przy odbieraniu pakietu, nr bledu: %d", errno);
            break;
        } else {
            rx_buffer[len] = 0;
            ESP_LOGI("UDP Server", "Odebrano pakiet: %s", rx_buffer);
            processMessage(rx_buffer);
        }
    }

    if (sock != -1) {
        ESP_LOGE("UDP Server", "Wystapil blad gniazda, uruchom ponownie ESP32.");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

// task for handling UDP client
void udp_client_task(void *pvParameters) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(ConnectivityData.udpSendAddress);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(ConnectivityData.udpSendPort);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE("UDP Client", "Nie mozna utworzyc gniazda, nr bledu: %d. Uruchom ponownie ESP32.", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        const char *message = "Hello, UDP server!";
        int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE("UDP Client", "Blad przy wysylaniu pakietu, nr bledu: %d", errno);
        } else {
            ESP_LOGI("UDP Client", "Wyslano pakiet danych.");
        }

        // TODO: make the loop only send data when there's actually something to send. Don't loop pointlessly.
        // One of the possible solutions is using vTaskSuspend and vTaskResume.
        vTaskDelay(udpClientDelay / portTICK_PERIOD_MS);
    }

    if (sock != -1) {
        ESP_LOGE("UDP Client", "Wystapil blad gniazda, uruchom ponownie ESP32.");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

// prepare udp payload before sending
void udpPreparePayload(int subject){
    char messageBuffer[256];
    string messageBufferString;

    messageBufferString.append("1");
    messageBufferString.append(to_string(123).substr(0, 5));
    messageBufferString.append("2");
    strcpy(messageBuffer, messageBufferString.c_str());
    // udpSend(messageBuffer);
    messageBufferString = "";
}