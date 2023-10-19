#include <string>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";
const uint8_t udpReceiveDelay = 32; // 32ms = 30 fps

// tasks config
// wifi connection maintenance task
const uint16_t maintainWifiConnectionDelay = 200; // delay, ms
const uint8_t maintainWifiConnectionPriority = 3; // priority
const bool maintainWifiConnectionCore = 1; // core, 0-1
// UDP packet receiver task
const uint16_t receiveUDPPacketDelay = 50; // delay, ms
const uint8_t receiveUDPPacketPriority = 2; // priority
const bool receiveUDPPacketCore = 1; // core, 0-1
// attaching interrupts task, no delay due to the task not having a loop
const uint8_t attachInterruptsPriority = 1; // priority
const bool attachInterruptsCore = 0; // core, 0-1

const uint16_t pulsesPerRotation = 1400;

struct{ // 1st motor
    const int pin = 25; // PWM
    const int motorPolarization_IN1 = 26; // polarization 1
    const int motorPolarization_IN2 = 27; // polarization 2
    const int standby = 14; // standby
    const int encoderLeftPin = 32;
    const int encoderRightPin = 33;
} leftMotor;

struct{ // 2nd motor
    const uint8_t pin = 0; // PWM
    const uint8_t motorPolarization_IN1 = 4; // polarization 1
    const uint8_t motorPolarization_IN2 = 16; // polarization 2
    const uint8_t standby = 17; // standby
    const uint8_t encoderLeftPin = 16; // TEMP - CHANGE IN FINAL VERSION
    const uint8_t encoderRightPin = 16; // TEMP - CHANGE IN FINAL VERSION
} rightMotor;

struct{
    const char* udpSendAddress = "1.2.3.4";
    const uint16_t udpSendPort = 1234;
    const uint16_t udpBeginPort = 1234;
} ConnectivityData;