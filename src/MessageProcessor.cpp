#include <ctype.h> // isdigit
#include <Arduino.h>
#include <Config.hpp>
#include <Motors.hpp>

int getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* origMsg = msg;
    char* newMsg = msg;
    int key = getKey(msg);
    if(key) return;
    for(int i = 0; i < 17; i++) newMsg++;

    while (*newMsg!='E')
    {
        switch(*newMsg){
            case 'K':
                newMsg++;
                angle = getValue(newMsg);
                Serial.print("1=");
                Serial.println(angle);
                break;
            case 'S':
                newMsg++;
                speed = getValue(newMsg);
                Serial.print("2=");
                Serial.println(speed);
                break;
            case 'O':
                newMsg++;
                axis = getValue(newMsg);
                Serial.print("3=");
                Serial.println(axis);
                break;
            case 'R':
                newMsg++;
                radius = getValue(newMsg);
                Serial.print("4=");
                Serial.println(radius);
                break;
            default:
                break;
        }
        newMsg++;
    }

    driveVehicle();
}

int getValue(char* msg){
    int val=0;
    int tempVal;
    while (isdigit(*msg)){
        val=(val*10)+(*msg-'0');
        msg++;
    }
    return val;
}

int getKey(char* msg){
    int keyVal = 1;
    char keyWordLocal[18];
    for(int i = 0; i < 17; i++){
        keyWordLocal[i] = *msg;
        msg++;
    }
    keyWordLocal[17] = '\0';
    if(!strcmp(keyWordLocal, keyWord)) keyVal = 0;
    return keyVal;
}