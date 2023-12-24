#ifndef ROACH_CONFIG_H
#define ROACH_CONFIG_H
#include "Controller.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>

using namespace std;

struct taskConfig{
  const uint16_t taskLoopDelay;
  const uint16_t taskSecondLoopDelay;
  const uint8_t taskPriority;
  const bool taskCore;
  TaskHandle_t taskHandle;
};

struct connectivityConfig{
  const char* udpSendAddress;
  const uint16_t udpSendPort;
  const uint16_t udpReceivePort;
  const char keyWord[18];
};

struct pidConfig{
  pid_ctrl_parameter_t params;
  const uint16_t loopPeriod;
  int setPoint;
};

extern pidConfig distancePidConf;
extern pidConfig leftMotorPid;
extern pidConfig rightMotorPid;

extern connectivityConfig connData;

extern taskConfig printDotsWhileConnectingToWifiConfig;
extern taskConfig udpClientConfig;
extern taskConfig udpServerConfig;
extern taskConfig wifiServiceCheckConnectionConfig;
extern taskConfig motorServiceConfig;

// encoder config
extern const uint16_t pulsesPerRevolution;
extern const float pulsesPerCm;
extern const uint16_t pcntHighLimit;
extern const int16_t pcntLowLimit;
extern const gpio_num_t standbyPin;
extern const uint16_t pulsesPerPowerPercent;
extern int maxMotorSpeed;
extern const uint8_t outputErrorTolerance;

// LEDs pins
extern const gpio_num_t redLED;
extern const gpio_num_t greenLED;
extern const gpio_num_t blueLED;

#endif