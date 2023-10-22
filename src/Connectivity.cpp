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

bool disconnectAction = false;
bool connected = false;
bool dots = false;

TaskHandle_t xDotsHandle;

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

static void wifiEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI("Wi-Fi", "Polaczono z siecia wifi.");
            //udp.begin(ConnectivityData.udpBeginPort);
            disconnectAction = false;
            connected = true;
            vTaskDelete(xDotsHandle);
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            if(!disconnectAction){
                ESP_LOGW("Wi-Fi", "Brak polaczenia z wifi. Zatrzymywanie systemu.");
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

// char* udpReceive(){
//     char* packetBuffer = new char[256];
//     //int packetSize = udp.parsePacket();
//     int length;
//     // if(packetSize){
//     //     length = udp.read(packetBuffer, 255);
//     //     if(length >= 0 ) packetBuffer[length] = '\0';
//     }
//     return packetBuffer;
// }

// task for receiving UDP packets
void receiveUDPPacket(void *params){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(true){
        // char *pMsg = udpReceive();
        // if(*pMsg != 0) processMessage(pMsg);
        // delete[] pMsg;
        xTaskDelayUntil(&xLastWakeTime, receiveUDPPacketDelay);
    }
}

void udpSend(char bufferSend[]){
    //udp.beginPacket(ConnectivityData.udpSendAddress, ConnectivityData.udpSendPort);
    //udp.print(bufferSend);
    //udp.endPacket();
}


void udpPrepareMessage(int subject){
    char messageBuffer[256];
    string messageBufferString;

    messageBufferString.append("1");
    messageBufferString.append(to_string(123).substr(0, 5));
    messageBufferString.append("2");
    strcpy(messageBuffer, messageBufferString.c_str());
    udpSend(messageBuffer);
    messageBufferString = "";
}