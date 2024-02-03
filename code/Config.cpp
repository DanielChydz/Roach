#include "Config.hpp"

// connectivity config
connectivityConfig connData = {
    .udpSendAddress = "192.168.147.180",
    .udpSendPort = 4322,
    .udpReceivePort = 4321,
    .keyWord = "DC_Remote_Car_Key",
};

// task config
taskConfig printDotsWhileConnectingToWifiConfig = {
    .taskLoopDelay = 1000,
    .taskPriority = 5,
    .taskCore = 1,
    .taskHandle = NULL
};

taskConfig udpClientConfig = {
    .taskLoopDelay = 2000,
    .taskPriority = 2,
    .taskCore = 1,
};

taskConfig udpServerConfig = {
    .taskLoopDelay = 200,
    .taskPriority = 2,
    .taskCore = 1,
};

taskConfig wifiServiceCheckConnectionConfig = {
    .taskLoopDelay = 100,
    .taskSecondLoopDelay = 250,
    .taskPriority = 3,
    .taskCore = 1,
};

taskConfig motorServiceConfig = {
    .taskLoopDelay = 200,
    .taskPriority = 1,
    .taskCore = 0,
};

// general config
const uint16_t pulsesPerRevolution = 2800;
const float pulsesPerCm = pulsesPerRevolution / (2 * 3.141 * 1.5); // pulsesPerRevolution / (2 * pi * r), where r is wheel radius
const uint16_t pcntHighLimit = 2800;
const int16_t pcntLowLimit = -2800;
const gpio_num_t standbyPin = GPIO_NUM_14;
const uint16_t pulsesPerPowerPercent = 4; // per loop
const uint16_t maxPulsesPerPowerPercent = 571; // per loop
int maxMotorSpeed = 100;

// PID config
pidConfig leftMotorPid = {
    .params = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .max_output = pulsesPerPowerPercent * 100,
        .min_output = -pulsesPerPowerPercent * 100,
        .max_integral = 100,
        .min_integral = -100,
    },
    .setPoint = 0,
};

pidConfig leftMotorSyncPid = {
    .params = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .max_output = pulsesPerPowerPercent * 100,
        .min_output = -pulsesPerPowerPercent * 100,
        .max_integral = 100,
        .min_integral = -100,
    },
    .setPoint = 0,
};

pidConfig rightMotorPid = {
    .params = {
        .kp = 0,
        .ki = 0,
        .kd = 0,
        .max_output = pulsesPerPowerPercent * 100,
        .min_output = -pulsesPerPowerPercent * 100,
        .max_integral = 100,
        .min_integral = -100,
    },
    .loopPeriod = 50,
    .setPoint = 0,
};

// motor properties
motorProperties leftMotorProperties = {
    .pwmPin = GPIO_NUM_4,
    .motorPolarization_IN1 = GPIO_NUM_16,
    .motorPolarization_IN2 = GPIO_NUM_17,
    .encoderLeftPin = GPIO_NUM_18,
    .encoderRightPin = GPIO_NUM_19,
    .loopPulses = 0,
    .pulses = 0,
    .motorSpeed = 0,
    .motorDir = 0, // 0 = forward
    .mcpwmGenerator = MCPWM_GEN_A,
};

motorProperties rightMotorProperties = {
    .pwmPin = GPIO_NUM_25,
    .motorPolarization_IN1 = GPIO_NUM_26,
    .motorPolarization_IN2 = GPIO_NUM_27,
    .encoderLeftPin = GPIO_NUM_32,
    .encoderRightPin = GPIO_NUM_33,
    .loopPulses = 0,
    .pulses = 0,
    .motorSpeed = 0,
    .motorDir = 1, // 1 = forward
    .mcpwmGenerator = MCPWM_GEN_B,
};

// LEDs pins
const gpio_num_t redLED = GPIO_NUM_21;
const gpio_num_t greenLED = GPIO_NUM_23;
const gpio_num_t blueLED = GPIO_NUM_22;