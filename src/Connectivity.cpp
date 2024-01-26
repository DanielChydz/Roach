#include "Credentials.hpp"
#include "MessageProcessor.hpp"
#include <esp_wifi.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <string>
#include <sys/param.h>
#include <lwip/sockets.h>

bool connected = false;
bool disconnectAction = false;
bool dots = false;
bool firstBoot = true;
bool lastMeasure = false;

char messageBuffer[256];

void udpServerTask(void *args);
void udpClientTask(void *args);
void wifiServiceTask(void *args);
const char *udpPreparePayload();

// task printing dots to the console when waiting for the wifi to connect
void printDotsWhileConnectingToWifi(void *args){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, 300); // wait for debug messages to pass
    printf("Oczekiwanie na polaczenie z wifi");
    while(!connected){
        printf(".");
        vTaskDelayUntil(&xLastWakeTime, printDotsWhileConnectingToWifiConfig.taskLoopDelay / portTICK_PERIOD_MS);
    }
}

static void wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("Wi-Fi", "Polaczono z siecia wifi.");
            disconnectAction = false;

            ESP_ERROR_CHECK(gpio_set_level(redLED, false));
            ESP_ERROR_CHECK(gpio_set_level(greenLED, true));
            ESP_ERROR_CHECK(gpio_set_level(blueLED, false));

            vTaskDelete(printDotsWhileConnectingToWifiConfig.taskHandle);
            connected = true;
            if(!firstBoot){
                vTaskResume(udpClientConfig.taskHandle);
                vTaskResume(udpServerConfig.taskHandle);
            }
            if(firstBoot) firstBoot = false;
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if(!disconnectAction){
                ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Zatrzymywanie systemu.");

                if(connected){
                    vTaskSuspend(udpClientConfig.taskHandle);
                    vTaskSuspend(udpServerConfig.taskHandle);
                }
                xTaskCreatePinnedToCore(
                    printDotsWhileConnectingToWifi,      // Function that should be called
                    "printDotsWhileConnectingToWifi",    // Name of the task (for debugging)
                    10000,               // Stack size (bytes)
                    NULL,               // Parameter to pass
                    printDotsWhileConnectingToWifiConfig.taskPriority,                  // Task priority
                    &printDotsWhileConnectingToWifiConfig.taskHandle,               // Task handle
                    printDotsWhileConnectingToWifiConfig.taskCore          // Core you want to run the task on (0 or 1)
                );
                if(!firstBoot) stopLoop();

                ESP_ERROR_CHECK(gpio_set_level(redLED, true));
                ESP_ERROR_CHECK(gpio_set_level(greenLED, false));
                ESP_ERROR_CHECK(gpio_set_level(blueLED, false));
                
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
        wifiServiceCheckConnectionConfig.taskPriority,                  // Task priority
        &wifiServiceCheckConnectionConfig.taskHandle,               // Task handle
        wifiServiceCheckConnectionConfig.taskCore          // Core you want to run the task on (0 or 1)
    );
    while(!connected) vTaskDelay(10);
    xTaskCreatePinnedToCore(
        udpServerTask,      // Function that should be called
        "UDPServer",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        udpServerConfig.taskPriority,                  // Task priority
        &udpServerConfig.taskHandle,               // Task handle
        udpClientConfig.taskCore          // Core you want to run the task on (0 or 1)
    );
    xTaskCreatePinnedToCore(
        udpClientTask,      // Function that should be called
        "UDPClient",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        udpClientConfig.taskPriority,                  // Task priority
        &udpClientConfig.taskHandle,               // Task handle
        udpClientConfig.taskCore          // Core you want to run the task on (0 or 1)
    );
}

// task maintaining wifi connection
void wifiServiceTask(void *args){
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
            while(!connected) vTaskDelayUntil(&xLastWakeTime, wifiServiceCheckConnectionConfig.taskSecondLoopDelay / portTICK_PERIOD_MS);
        }
        vTaskDelayUntil(&xLastWakeTime, wifiServiceCheckConnectionConfig.taskLoopDelay / portTICK_PERIOD_MS);
    }
}

// task for handling UDP server
void udpServerTask(void *args) {
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
    server_addr.sin_port = htons(connData.udpReceivePort);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE("UDP Server", "Nie mozna przypiąć IP i portu do gniazda, nr bledu: %d. Uruchom ponownie ESP32.", errno);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
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
void udpClientTask(void *args) {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(connData.udpSendAddress);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(connData.udpSendPort);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE("UDP Client", "Nie mozna utworzyc gniazda, nr bledu: %d. Uruchom ponownie ESP32.", errno);
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const char *message = udpPreparePayload();
        int err = sendto(sock, message, strlen(message), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if(errno == 12){
            ESP_LOGW("UDP Client", "Urządzenie docelowe jest offline, nie można wysłać pakietu.");
         } else if(err < 0){
            ESP_LOGE("UDP Client", "Blad przy wysylaniu pakietu, nr bledu: %d", errno);
        } else {
            ESP_LOGI("UDP Client", "Wyslano pakiet danych.");
        }
    }

    if (sock != -1) {
        ESP_LOGE("UDP Client", "Wystapil blad gniazda, uruchom ponownie ESP32.");
        shutdown(sock, 0);
        close(sock);
    }
    vTaskDelete(NULL);
}

// prepare udp payload before sending
const char *udpPreparePayload(){
    string messageBufferString;

    messageBufferString.append("DC_Remote_Car_Key");
    if(!lastMeasure){
        // set point
        messageBufferString.append("A");
        messageBufferString.append(to_string(rightMotorPid.setPoint));
        // left motor loop pulses
        messageBufferString.append("B");
        messageBufferString.append(to_string(leftMotorProperties.loopPulses));
        // right motor loop pulses
        messageBufferString.append("C");
        messageBufferString.append(to_string(rightMotorProperties.loopPulses));
        // left motor pulses
        messageBufferString.append("D");
        messageBufferString.append(to_string(leftMotorProperties.pulses));
        // right motor pulses
        messageBufferString.append("E");
        messageBufferString.append(to_string(rightMotorProperties.pulses));
        // left motor speed
        messageBufferString.append("F");
        messageBufferString.append(to_string(leftMotorProperties.motorSpeed / (pulsesPerPowerPercent * 100.0 / maxPulsesPerPowerPercent)));
        // right motor speed
        messageBufferString.append("G");
        messageBufferString.append(to_string(rightMotorProperties.motorSpeed / (pulsesPerPowerPercent * 100.0 / maxPulsesPerPowerPercent)));
        // max motor speed
        messageBufferString.append("H");
        messageBufferString.append(to_string(maxMotorSpeed));
        // max impulses per loop
        messageBufferString.append("I");
        messageBufferString.append(to_string(pulsesPerPowerPercent * 100));
    } else {
        messageBufferString.append("Z");
    }
    strcpy(messageBuffer, messageBufferString.c_str());
    return messageBuffer;
}