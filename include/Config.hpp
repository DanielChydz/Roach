#include <string>
#include <driver/gpio.h>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";
const uint8_t udpReceiveDelay = 32; // 32ms = 30 fps

// tasks config
// wifi connection maintenance task
const uint16_t printDotsWhileConnectingToWifiDelay = 1000; // delay, ms
const uint8_t printDotsWhileConnectingToWifiPriority = 5; // priority
const bool printDotsWhileConnectingToWifiCore = 1; // core, 0-1
// UDP client task
const uint16_t UDPClientDelay = 200; // delay, ms
const uint8_t UDPClientPriority = 2; // priority
const bool UDPClientCore = 1; // core, 0-1
// attaching interrupts task, no delay due to the task not having a loop
const uint8_t attachInterruptsPriority = 1; // priority
const bool attachInterruptsCore = 0; // core, 0-1

const uint16_t pulsesPerRotation = 1400;

struct{ // 1st motor
    const gpio_num_t pin = GPIO_NUM_25; // PWM
    const gpio_num_t motorPolarization_IN1 = GPIO_NUM_26; // polarization 1
    const gpio_num_t motorPolarization_IN2 = GPIO_NUM_27; // polarization 2
    const gpio_num_t standby = GPIO_NUM_14; // standby
    const gpio_num_t encoderLeftPin = GPIO_NUM_32;
    const gpio_num_t encoderRightPin = GPIO_NUM_33;
} leftMotor;

struct{ // 2nd motor
    const gpio_num_t pin = GPIO_NUM_0; // PWM
    const gpio_num_t motorPolarization_IN1 = GPIO_NUM_4; // polarization 1
    const gpio_num_t motorPolarization_IN2 = GPIO_NUM_16; // polarization 2
    const gpio_num_t standby = GPIO_NUM_17; // standby
    const gpio_num_t encoderLeftPin = GPIO_NUM_16; // TEMP - CHANGE IN FINAL VERSION
    const gpio_num_t encoderRightPin = GPIO_NUM_16; // TEMP - CHANGE IN FINAL VERSION
} rightMotor;

struct{
    const char* udpSendAddress = "1.2.3.4";
    const uint16_t udpSendPort = 1234;
    const uint16_t udpBeginPort = 1234;
} ConnectivityData;