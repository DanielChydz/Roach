#include <string>
#include <driver/gpio.h>

using namespace std;

const char keyWord[18] = "DC_Remote_Car_Key";

//Tasks config.
// printing dots while waiting for wifi connection task
const uint16_t printDotsWhileConnectingToWifiDelay = 1000; // delay, ms
const uint8_t printDotsWhileConnectingToWifiPriority = 5; // priority
const bool printDotsWhileConnectingToWifiCore = 1; // core, 0-1
// UDP client task
const uint16_t udpClientDelay = 2000; // delay, ms
const uint8_t udpClientPriority = 2; // priority
const bool udpClientCore = 1; // core, 0-1
// UDP server task
const uint16_t udpServerDelay = 200; // delay, ms
const uint8_t udpServerPriority = 2; // priority
const bool udpServerCore = 1; // core, 0-1
// wifi service task
const uint16_t wifiServiceCheckConnectionDelay = 100; // delay, ms
const uint16_t wifiServiceWaitForConnectionDelay = 250; // delay, ms
const uint8_t wifiServicePriority = 3; // priority
const bool wifiServiceCore = 1; // core, 0-1
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
    const char* udpSendAddress = "192.168.220.201";
    const uint16_t udpSendPort = 4322;
    const uint16_t udpReceivePort = 4321;
} ConnectivityData;