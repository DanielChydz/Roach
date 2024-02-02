#include "MessageProcessor.hpp"

float getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* newMsg = msg;
    bool key = getKey(msg);
    bool stopMeasurement = false;
    float temp;
    if(!key) return;
    for(int i = 0; i < 17; i++) newMsg++; // 17 should be calculated on the go instead of hard coded

    while (*newMsg != 'Z')
    {
        switch(*newMsg){
            // set point
            case 'A':
                newMsg++;
                maxMotorSpeed = getValue(newMsg);
                continue;
            // left motor PID Kp
            case 'B':
                newMsg++;
                leftMotorPid.params.kp = getValue(newMsg);
                continue;
            // left motor PID Ki
            case 'C':
                newMsg++;
                leftMotorPid.params.ki = getValue(newMsg);
                continue;
            // left motor PID Kd
            case 'D':
                newMsg++;
                leftMotorPid.params.kd = getValue(newMsg);
                continue;
            // right motor PID Kp
            case 'E':
                newMsg++;
                rightMotorPid.params.kp = getValue(newMsg);
                continue;
            // right motor PID Ki
            case 'F':
                newMsg++;
                rightMotorPid.params.ki = getValue(newMsg);
                continue;
            // right motor PID Kd
            case 'G':
                newMsg++;
                rightMotorPid.params.kd = getValue(newMsg);
                continue;
            // sync PID Kp
            case 'H':
                newMsg++;
                leftMotorSyncPid.params.kp = getValue(newMsg);
                continue;
            // sync PID Kd
            case 'I':
                newMsg++;
                leftMotorSyncPid.params.ki = getValue(newMsg);
                continue;
            // sync PID Kd
            case 'J':
                newMsg++;
                leftMotorSyncPid.params.kd = getValue(newMsg);
                continue;
            // set point
            case 'K':
                newMsg++;
                rightMotorPid.setPoint = getValue(newMsg) * pulsesPerCm;
                continue;
            //
            //
            //
            // end measurement
            case 'Q':
                newMsg++;
                stopMeasurement = true;
                continue;
            default:
                newMsg++;
        }
    }
    
    if(stopMeasurement){
        stopLoop();
    }else if(!executingTask){
        startLoop();
    }
}

// extract values from data
float getValue(char* msg){
    char* endPtr;
    return strtof(msg, &endPtr);
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
    if(strcmp(keyWordLocal, connData.keyWord) == 0) keyVal = true;
    return keyVal;
}