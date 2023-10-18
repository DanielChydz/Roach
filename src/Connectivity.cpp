#include <Arduino.h>
#include <Config.hpp>
#include <Credentials.hpp>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MessageProcessor.hpp>
#include <Regulator.hpp>

WiFiUDP udp;
bool firstBoot = true;
bool connected = false;

// task for maintaining wifi connection
void maintainWifiConnection(void *param){
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true){
        if(WiFi.status() != WL_CONNECTED){
            connected = false;
            brake();

            if(firstBoot)Serial.println("\nRozlaczono z siecia wifi.");
        
            WiFi.begin(ssid, password);
            Serial.println("\nLaczenie z wifi.");
        
            while(WiFi.status() != WL_CONNECTED){
                Serial.print(".");
                vTaskDelayUntil( &xLastWakeTime, 1000 )
            }

            Serial.println("\nPolaczono z siecia wifi.");
            Serial.print("Adres lokalny ESP32: ");
            Serial.println(WiFi.localIP());
            connected = true;
        }
        if(!firstBoot) {
            udp.begin(ConnectivityData.udpBeginPort);
            firstBoot = false;
        }
        xTaskDelayUntil(&xLastWakeTime, maintainWifiConnectionDelay);
    }
}

char* udpReceive(){
    char* packetBuffer = new char[256];
    int packetSize = udp.parsePacket();
    int length;
    if(packetSize){
        length = udp.read(packetBuffer, 255);
        if(length >= 0 ) packetBuffer[length] = '\0';
    }
    return packetBuffer;
}

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
    udp.beginPacket(ConnectivityData.udpSendAddress, ConnectivityData.udpSendPort);
    udp.print(bufferSend);
    udp.endPacket();
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