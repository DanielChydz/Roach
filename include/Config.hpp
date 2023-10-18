#include <string>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";
const int udpReceiveDelay = 32; // 30 fps

// tasks config
// wifi connection maintenance task
const int maintainWifiConnectionDelay = 200; // delay, ms
const int maintainWifiConnectionPriority = 3; // priority
const int maintainWifiConnectionCore = 1; // core, 0-1
// UDP packet receiver task
const int receiveUDPPacketDelay = 50; // delay, ms
const int receiveUDPPacketPriority = 2; // priority
const int receiveUDPPacketCore = 1; // core, 0-1
// attaching interrupts task, no delay due to the task not having a loop
const int attachInterruptsPriority = 1; // priority
const int attachInterruptsCore = 0; // core, 0-1

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
    const int encoderLeftPin = 16; // TEMP - CHANGE IN FINAL VERSION
    const int encoderRightPin = 16; // TEMP - CHANGE IN FINAL VERSION
} secondMotor;

struct{
    const char* udpSendAddress = "1.2.3.4";
    const int udpSendPort = 1234;
    const int udpBeginPort = 1234;
} ConnectivityData;