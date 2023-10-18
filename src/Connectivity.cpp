#include <Arduino.h>
#include <Config.hpp>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <MessageProcessor.hpp>
#include <Regulator.hpp>

WiFiUDP udp;
bool firstBoot = false;
bool connected = false;

void connectWifi(){
    bool isPrinted = false;

    if(WiFi.status() != WL_CONNECTED){
        connected = false;
        brake();

        if(!isPrinted && firstBoot) {
            Serial.println("\nRozlaczono z siecia wifi.");
            isPrinted = true;
        }
        
        WiFi.begin(ConnectivityData.ssid, ConnectivityData.password);
        Serial.println("\nLaczenie z wifi.");
        
        while(WiFi.status() != WL_CONNECTED){
            Serial.print(".");
            delay(1000);
        }

        isPrinted = false;
        firstBoot = true;

        Serial.println("\nPolaczono z siecia wifi.");
        Serial.print("Adres lokalny ESP32: ");
        Serial.println(WiFi.localIP());
        connected = true;
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

void udpSend(char bufferSend[]){
    udp.beginPacket(ConnectivityData.udpSendAddress, ConnectivityData.udpSendPort);
    udp.print(bufferSend);
    udp.endPacket();
}

// 1 = proximity, 2 = gyroscope, 3 = accelerometer;
// void udpPrepareMessage(int subject){
//     char messageBuffer[256];
//     string messageBufferString;
//     switch(subject){
//         case 1:
//             messageBufferString.append("DBG:PX1=");
//             messageBufferString.append(to_string(distanceFirst).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";
            
//             messageBufferString.append("DBG:PX2=");
//             messageBufferString.append(to_string(distanceSecond).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             messageBufferString.append("DBG:PX3=");
//             messageBufferString.append(to_string(distanceThird).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             messageBufferString.append("DBG:PX4=");
//             messageBufferString.append(to_string(distanceFourth).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             break;
//         case 2:
//             messageBufferString.append("DBG:GYX=");
//             messageBufferString.append(to_string(gyroX).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             messageBufferString.append("DBG:GYY=");
//             messageBufferString.append(to_string(gyroY).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             messageBufferString.append("DBG:GYZ=");
//             messageBufferString.append(to_string(gyroZ).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";
//             break;
//         case 3:
//             messageBufferString.append("DBG:ACX=");
//             messageBufferString.append(to_string(accX).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";

//             messageBufferString.append("DBG:ACY=");
//             messageBufferString.append(to_string(accY).substr(0, 5));
//             messageBufferString.append(";");
//             strcpy(messageBuffer, messageBufferString.c_str());
//             udpSend(messageBuffer);
//             messageBufferString = "";
//             break;
//         default:
//             break;
//     }
// }

void maintainWifiConnection(){
    connectWifi();
}

void receiveUDPRoutine(){
        char *pMsg = udpReceive();
        if(*pMsg != 0) processMessage(pMsg);
        delete[] pMsg;
}