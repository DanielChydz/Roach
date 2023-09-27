#include <string>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";
const int udpReceiveDelay = 32; // 30 fps
const int wifiReconnectRoutine = 500; // 

struct{ // 1st motor
    const int pin = 19; // PWM
    const int motorPolarization_IN1 = 5; // polarization 1
    const int motorPolarization_IN2 = 18; // polarization 2
    const int standby = 17; // standby
} firstMotor;

struct{ // 2nd motor
    const int pin = 0; // PWM
    const int motorPolarization_IN1 = 4; // polarization 1
    const int motorPolarization_IN2 = 16; // polarization 2
    const int standby = 17; // standby
} secondMotor;

struct{ // proximity sensor
    const int trigPin = 32; // trigger
    const int echoPin = 33; // echo
} proximitySensor;

struct{
    const char* ssid = "defaultSSID";
    const char* password = "defaultPassword";
    const char* udpSendAddress = "1.2.3.4";
    const int udpSendPort = 1234;
    const int udpBeginPort = 1234;
} ConnectivityData;