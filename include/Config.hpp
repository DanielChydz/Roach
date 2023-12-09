#ifndef ROACH_CONFIG_H
#define ROACH_CONFIG_H
#include "Controller.hpp"
#include <string>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace std;

struct taskConfig{
  const uint16_t taskLoopDelay;
  const uint16_t taskSecondLoopDelay;
  const uint8_t taskPriority;
  const bool taskCore;
  TaskHandle_t taskHandle;
};

struct connectivityData{
  const char* udpSendAddress;
  const uint16_t udpSendPort;
  const uint16_t udpReceivePort;
};

extern connectivityData connData;

extern taskConfig printDotsWhileConnectingToWifiConfig;
extern taskConfig udpClientConfig;
extern taskConfig udpServerConfig;
extern taskConfig wifiServiceCheckConnectionConfig;
extern taskConfig motorServiceConfig;

extern const char keyWord[18];

// encoder config
const uint16_t pulsesPerRevolution = 1400;
const uint8_t pulsesPerCm = pulsesPerRevolution / (2 * 3.141 * 1.5); // pulsesPerRevolution / (2 * pi * r), where r is wheel radius
const uint16_t pcntHighLimit = 1400;
const int16_t pcntLowLimit = -1400;

// pid config
const uint16_t loopPeriod = 20; // how often motor speed is supposed to be calculated, ms
const float pidKp = 100;
const float pidKi = 50;
const float pidKd = 20;

// motor driver standby pin
const gpio_num_t standbyPin = GPIO_NUM_14;

#endif