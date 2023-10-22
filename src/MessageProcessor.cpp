#include <ctype.h> // isdigit
#include <Config.hpp>
#include <Controller.hpp>
#include <string.h> // strcmp

int getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* newMsg = msg;
    int key = getKey(msg);
    if(key) return;
    for(int i = 0; i < 17; i++) newMsg++; // 17 should be calculated on the go instead of hard coded

    while (*newMsg!='E')
    {
        switch(*newMsg){
            // forward
            case 'P':
                newMsg++;
                //angle = getValue(newMsg);
                break;
            // backwards
            case 'T' || 'B':
                newMsg++;
                //speed = getValue(newMsg);
                break;
            // right
            case 'R':
                newMsg++;
                // = getValue(newMsg);
                break;
            // left
            case 'L':
                newMsg++;
                // = getValue(newMsg);
                break;
            case 'S':
                newMsg++;
                // = getValue(newMsg);
            default:
                break;
        }
        newMsg++;
    }

    driveVehicle();
}

// extract values from data
int getValue(char* msg){
    int val=0;
    while (isdigit(*msg)){
        val=(val*10)+(*msg-'0');
        msg++;
    }
    return val;
}

// check if data contains key word
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