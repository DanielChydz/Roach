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
            // both wheels
            case 'A':
                newMsg++;
                distancePidConf.setPoint = getValue(newMsg) * pulsesPerCm;
                continue;
            // left wheel
            case 'B':
                newMsg++;
                leftMotorProperties.distance = getValue(newMsg) * pulsesPerCm;
                if(leftMotorProperties.distance > 0){
                    leftMotorProperties.motorDir = 0;
                } else if(leftMotorProperties.distance < 0){
                    leftMotorProperties.motorDir = 1;
                }
                continue;
            // right wheel
            case 'C':
                newMsg++;
                rightMotorProperties.distance = getValue(newMsg) * pulsesPerCm;
                continue;
            // speed
            case 'D':
                newMsg++;
                maxMotorSpeed = getValue(newMsg);
                temp = pulsesPerRevolution * maxMotorSpeed * 0.01;
                distancePidConf.params.max_integral = 200;
                leftMotorPid.params.max_integral = 200;
                rightMotorPid.params.max_integral = 200;
                distancePidConf.params.min_integral = -200;
                leftMotorPid.params.min_integral = -200;
                rightMotorPid.params.min_integral = -200;
                distancePidConf.params.max_output = distancePidConf.setPoint; // 10000 * maxMotorSpeed * 0.01
                leftMotorPid.params.max_output = distancePidConf.setPoint;
                rightMotorPid.params.max_output = distancePidConf.setPoint;
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
    float val=0;
    bool negative = false;
    bool fraction = false;
    while (isdigit(*msg) || *msg == '-' || *msg == '.'){
        if(isdigit(*msg)){
            if(!fraction){
                val = (val*10)+(*msg-'0');
            } else{
                val += (*msg-'0')/10.0;
            }
        } else if(*msg == '-'){
            negative = true;
        } else {
            fraction = true;
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