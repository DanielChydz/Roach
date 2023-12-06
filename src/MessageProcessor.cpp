#include <ctype.h> // isdigit
#include <Config.hpp>
#include <Controller.hpp>
#include <string.h> // strcmp
#include <vector>

int getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* newMsg = msg;
    bool key = getKey(msg);
    int temp = 0;
    if(!key) return;
    for(int i = 0; i < 17; i++) newMsg++; // 17 should be calculated on the go instead of hard coded

    while (*newMsg != 'E')
    {
        switch(*newMsg){
            // both wheels
            case 'B':
                newMsg++;
                temp = getValue(newMsg);
                if(temp > 0){
                    bothWheelsValue = temp;
                    leftMotorDir = 0;
                    rightMotorDir = 1;
                } else if(temp < 0){
                    bothWheelsValue = temp * (-1);
                    leftMotorDir = 1;
                    rightMotorDir = 0;
                }
                continue;
            // left wheel
            case 'L':
                newMsg++;
                temp = getValue(newMsg);
                if(temp > 0){
                    leftWheelValue = temp;
                    leftMotorDir = 0;
                } else if(temp < 0){
                    leftWheelValue = temp * (-1);
                    leftMotorDir = 1;
                }
                continue;
            // right wheel
            case 'R':
                newMsg++;
                temp = getValue(newMsg);
                if(temp > 0){
                    rightWheelValue = temp;
                    rightMotorDir = 0;
                } else if(temp < 0){
                    rightWheelValue = temp * (-1);
                    rightMotorDir = 1;
                }
                continue;
            // speed
            case 'S':
                newMsg++;
                speedSetPoint = getValue(newMsg);
                continue;
            // unit
            case 'U':
                newMsg++;
                unit = getValue(newMsg);
                continue;
            default:
                newMsg++;
        }
    }

    driveVehicle();
}

// extract values from data
int getValue(char* msg){
    int val=0;
    bool negative = false;
    while (isdigit(*msg) || *msg == '-'){
        if(*msg != '-'){
            val=(val*10)+(*msg-'0');
        } else {
            negative = true;
        }
        msg++;
    }
    if(negative) val *= -1;
    return val;
}

// check if data contains key word
int getKey(char* msg){
    bool keyVal = false;
    char keyWordLocal[18];
    for(int i = 0; i < 17; i++){
        keyWordLocal[i] = *msg;
        msg++;
    }
    keyWordLocal[17] = '\0';
    if(strcmp(keyWordLocal, keyWord) == 0) keyVal = true;
    return keyVal;
}