#include "MessageProcessor.hpp"

int getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* newMsg = msg;
    bool key = getKey(msg);
    int temp = 0;
    if(!key || executingTask) return;
    for(int i = 0; i < 17; i++) newMsg++; // 17 should be calculated on the go instead of hard coded

    while (*newMsg != 'Z')
    {
        switch(*newMsg){
            // both wheels
            case 'A':
                newMsg++;
                distancePidConf.setPoint = getValue(newMsg);
                continue;
            // left wheel
            case 'B':
                newMsg++;
                temp = getValue(newMsg);
                if(temp > 0){
                    leftMotorProperties.distance = temp;
                    leftMotorProperties.motorDir = 0;
                } else if(temp < 0){
                    rightMotorProperties.distance = temp * (-1);
                    leftMotorProperties.motorDir = 1;
                }
                continue;
            // right wheel
            case 'C':
                newMsg++;
                temp = getValue(newMsg);
                if(temp > 0){
                    rightMotorProperties.distance = temp;
                    rightMotorProperties.motorDir = 1;
                } else if(temp < 0){
                    rightMotorProperties.distance = temp * (-1);
                    rightMotorProperties.motorDir = 0;
                }
                continue;
            // speed
            case 'D':
                newMsg++;
                maxMotorSpeed = getValue(newMsg);
                continue;
            // distance PID Kp
            case 'E':
                newMsg++;
                distancePidConf.params.kp = getValue(newMsg);
                continue;
            // distance PID Ki
            case 'F':
                newMsg++;
                distancePidConf.params.ki = getValue(newMsg);
                continue;
            // distance PID Kd
            case 'G':
                newMsg++;
                distancePidConf.params.kd = getValue(newMsg);
                continue;
            // left motor PID Kp
            case 'H':
                newMsg++;
                leftMotorPid.params.kp = getValue(newMsg);
                continue;
            // left motor PID Ki
            case 'I':
                newMsg++;
                leftMotorPid.params.ki = getValue(newMsg);
                continue;
            // left motor PID Kd
            case 'J':
                newMsg++;
                leftMotorPid.params.kd = getValue(newMsg);
                continue;
            // right motor PID Kp
            case 'K':
                newMsg++;
                rightMotorPid.params.kp = getValue(newMsg);
                continue;
            // right motor PID Ki
            case 'L':
                newMsg++;
                rightMotorPid.params.ki = getValue(newMsg);
                continue;
            // right motor PID Kd
            case 'M':
                newMsg++;
                rightMotorPid.params.kd = getValue(newMsg);
                continue;
            default:
                newMsg++;
        }
    }
    
    if(!executingTask) {
        pid_update_parameters(leftMotorProperties.pidCtrl, &leftMotorPid.params);
        pid_update_parameters(rightMotorProperties.pidCtrl, &rightMotorPid.params);
        pid_update_parameters(pidHandle, &distancePidConf.params);
        startLoop();
    }
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
    if(strcmp(keyWordLocal, connData.keyWord) == 0) keyVal = true;
    return keyVal;
}