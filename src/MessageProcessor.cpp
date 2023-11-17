#include <ctype.h> // isdigit
#include <Config.hpp>
#include <Controller.hpp>
#include <string.h> // strcmp
#include <vector>

vector<char> getValue(char* msg);
int getKey(char* msg);

void processMessage(char* msg){
    char* newMsg = msg;
    int key = getKey(msg);
    if(key) return;
    for(int i = 0; i < 17; i++) newMsg++; // 17 should be calculated on the go instead of hard coded

    while (*newMsg != 'E')
    {
        switch(*newMsg){
            // IP
            case 'I':
                newMsg++;
                ConnectivityData.udpSendAddress = reinterpret_cast<char*>(getValue(newMsg).data());;
                printf(ConnectivityData.udpSendAddress);
                break;
            // both wheels
            case 'B' || 'B':
                newMsg++;
                //speed = getValue(newMsg);
                break;
            // left wheel
            case 'L':
                newMsg++;
                // = getValue(newMsg);
                break;
            // right wheel
            case 'R':
                newMsg++;
                // = getValue(newMsg);
                break;
            // speed
            case 'S':
                newMsg++;
                // = getValue(newMsg);
                break;
            // unit
            case 'U':
                newMsg++;
                // = getValue(newMsg);
            default:
                break;
        }
        newMsg++;
    }

    // driveVehicle();
}

// extract values from data
vector<char> getValue(char* msg){
    vector<char> val_test;
    int val=0;
    while (isdigit(*msg) || *msg == '.'){
        //val=(val*10)+(*msg-'0');
        val_test.push_back(*msg);
        msg++;
    }
    return val_test;
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