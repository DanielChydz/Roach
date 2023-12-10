#ifndef ROACH_CONFIG_H
#define ROACH_CONFIG_H
#include "Controller.hpp"
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

struct connectivityConfig{
  const char* udpSendAddress;
  const uint16_t udpSendPort;
  const uint16_t udpReceivePort;
  const char keyWord[18];
};

struct pidConfig{
  uint16_t setPoint;
  const uint16_t loopPeriod;
  const float pidKp;
  const float pidKi;
  const float pidKd;
  const float maxOutput;
  const float minOutput;
  const float maxIntegral;
  const float minIntegral;
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
extern int maxMotorSpeed;

#endif