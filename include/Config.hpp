#include <string>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";
const int udpReceiveDelay = 32; // 30 fps
const int wifiReconnectRoutine = 500; // ? 

const int pulsesPerRotation = 1400;

struct{ // 1st motor
    const int pin = 25; // PWM
    const int motorPolarization_IN1 = 26; // polarization 1
    const int motorPolarization_IN2 = 27; // polarization 2
    const int standby = 14; // standby
    const int encoderLeftPin = 32;
    const int encoderRightPin = 33;
} firstMotor;

struct{ // 2nd motor
    const int pin = 0; // PWM
    const int motorPolarization_IN1 = 4; // polarization 1
    const int motorPolarization_IN2 = 16; // polarization 2
    const int standby = 17; // standby
    const int encoderLeftPin = -1; // TEMP - CHANGE IN FINAL VERSION
    const int encoderRightPin = -1; // TEMP - CHANGE IN FINAL VERSION
} secondMotor;

struct{
    const char* ssid = "defaultSSID";
    const char* password = "defaultPassword";
    const char* udpSendAddress = "1.2.3.4";
    const int udpSendPort = 1234;
    const int udpBeginPort = 1234;
} ConnectivityData;