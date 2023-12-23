#include "Config.hpp"
#include <esp_log.h>
#include <esp_timer.h>
#include <sys/param.h>
#include <cmath>
#include <deque>
#include <numeric>
#include <algorithm>

bool executingTask = false;
static int lastPulseCountLeftMotor = 0;
static int lastPulseCountRightMotor = 0;
static deque<int> pidOutput;
static deque<int> pidOutputLeft;
static deque<int> pidOutputRight;
static esp_timer_handle_t loopTimer = NULL;
static pid_ctrl_block_handle_t pidHandle = NULL;

measurementData measurements;

void motorServiceTask(void *args);
void driveMotor(motorProperties *args);
void configureMotor(motorProperties *args);
std::pair<float, float> calculateMeanAndSampleStandardDeviation(const std::deque<int>& data);

void stopLoop(){
  executingTask = false;
  esp_timer_stop(loopTimer);
  lastPulseCountLeftMotor = 0;
  lastPulseCountRightMotor = 0;
  leftMotorProperties.motorSpeed = 0;
  rightMotorProperties.motorSpeed = 0;
  pidOutput.clear();
  pidOutputLeft.clear();
  pidOutputRight.clear();
  driveMotor(&leftMotorProperties);
  driveMotor(&rightMotorProperties);
  ESP_LOGI("PID", "Wynik petli =============================================");
  ESP_LOGI("PID", "Lewy silnik: %d", abs(leftMotorProperties.pulses));
  ESP_LOGI("PID", "Prawy silnik: %d", abs(rightMotorProperties.pulses));
  ESP_LOGI("PID", "Srednia: %d", (abs(leftMotorProperties.pulses) + abs(rightMotorProperties.pulses))/2);
  ESP_LOGI("PID", "Cel: %d", abs(distancePidConf.setPoint));
  lastMeasure = true;
  xTaskNotifyGive(udpClientConfig.taskHandle);
  vTaskDelay(pdMS_TO_TICKS(50));
  measurements.lowerThreshold = 0;
  measurements.upperThreshold = 0;
  measurements.mean = 0;
  measurements.standardDeviation = 0;
  leftMotorProperties.pulses = 0;
  rightMotorProperties.pulses = 0;
  pcnt_unit_clear_count(leftMotorProperties.pcntUnit);
  pcnt_unit_clear_count(rightMotorProperties.pcntUnit);
  pid_reset_ctrl_block(leftMotorProperties.pidCtrl);
  pid_reset_ctrl_block(rightMotorProperties.pidCtrl);
  pid_reset_ctrl_block(pidHandle);
}

void startLoop(){
  ESP_LOGI("PID", "Poczatek petli =============================================");
  ESP_LOGI("PID", "Cel: %d pulsow", distancePidConf.setPoint);
  pid_update_parameters(leftMotorProperties.pidCtrl, &leftMotorPid.params);
  pid_update_parameters(rightMotorProperties.pidCtrl, &rightMotorPid.params);
  pid_update_parameters(pidHandle, &distancePidConf.params);
  
  executingTask = true;
  lastMeasure = false;
  lastPulseCountLeftMotor = 0;
  lastPulseCountRightMotor = 0;
  leftMotorProperties.pulses = 0;
  rightMotorProperties.pulses = 0;
  esp_timer_start_periodic(loopTimer, distancePidConf.loopPeriod * 1000);
}

static void motorLoop(void *args){
  pid_ctrl_block_handle_t *pidCtrl = (pid_ctrl_block_handle_t *)args;
  int mean = (abs(leftMotorProperties.pulses) + abs(rightMotorProperties.pulses))/2;
  pcnt_unit_get_count(leftMotorProperties.pcntUnit, &leftMotorProperties.pulses);
  pcnt_unit_get_count(rightMotorProperties.pcntUnit, &rightMotorProperties.pulses);
  leftMotorProperties.loopPulses = leftMotorProperties.pulses - lastPulseCountLeftMotor;
  rightMotorProperties.loopPulses = rightMotorProperties.pulses - lastPulseCountRightMotor;
  lastPulseCountLeftMotor = leftMotorProperties.pulses;
  lastPulseCountRightMotor = rightMotorProperties.pulses;
  if(mean > distancePidConf.setPoint - 1000){
    pidOutput.push_front(mean);
    pidOutputLeft.push_front(abs(leftMotorProperties.pulses));
    pidOutputRight.push_front(abs(rightMotorProperties.pulses));
    if(pidOutput.size() > 20) pidOutput.pop_back();
    if(pidOutputLeft.size() > 20) pidOutputLeft.pop_back();
    if(pidOutputRight.size() > 20) pidOutputRight.pop_back();
  }

  xTaskNotifyGive(udpClientConfig.taskHandle);
  xTaskNotifyGive(motorServiceConfig.taskHandle);

  float errorDistance = abs(distancePidConf.setPoint) - mean;
  errorDistance = MAX(MIN(errorDistance, 10000), -10000);
  float motorPower = 0;
  pid_compute(*pidCtrl, errorDistance, &motorPower);

  // float errorLeftMotor = motorPower - abs(leftMotorProperties.loopPulses);
  // float errorRightMotor = motorPower - abs(rightMotorProperties.loopPulses);
  // pid_compute(leftMotorProperties.pidCtrl, errorLeftMotor, &leftMotorProperties.motorSpeed);
  // pid_compute(rightMotorProperties.pidCtrl, errorRightMotor, &rightMotorProperties.motorSpeed);
  leftMotorProperties.motorSpeed = motorPower / pulsesPerPowerPercent;
  rightMotorProperties.motorSpeed = motorPower / pulsesPerPowerPercent;

  if((distancePidConf.setPoint > 0 && errorDistance > 0) || (distancePidConf.setPoint < 0 && errorDistance < 0)){
    leftMotorProperties.motorDir = 0;
    rightMotorProperties.motorDir = 1;
  } else if((distancePidConf.setPoint > 0 && errorDistance < 0) || (distancePidConf.setPoint < 0 && errorDistance > 0)){
    leftMotorProperties.motorDir = 1;
    rightMotorProperties.motorDir = 0;
  }

  ESP_LOGI("PID", "error: %f", errorDistance);
  ESP_LOGI("PID", "motorPower: %f", motorPower);
  ESP_LOGI("PID", "left pulses: %d", leftMotorProperties.pulses);
  ESP_LOGI("PID", "right pulses: %d", rightMotorProperties.pulses);
  ESP_LOGI("PID", "motorSpeed: %f", leftMotorProperties.motorSpeed);
  ESP_LOGI("PID", "motorSpeed: %f", rightMotorProperties.motorSpeed);

  driveMotor(&leftMotorProperties);
  driveMotor(&rightMotorProperties);

  // ESP_LOGI("PID", "Lewy silnik: %f", leftMotorProperties.motorSpeed);
  // ESP_LOGI("PID", "Prawy silnik: %f", rightMotorProperties.motorSpeed);
  // ESP_LOGI("PID", "Lewy silnik: %d na %d", abs(leftMotorProperties.pulses), abs(distancePidConf.setPoint));
  // ESP_LOGI("PID", "Prawy silnik: %d na %d", abs(rightMotorProperties.pulses), abs(distancePidConf.setPoint));
}

void startMotorService() {
  ESP_LOGI("Controller", "Konfiguracja silnikow.");

  gpio_set_direction(standbyPin, GPIO_MODE_OUTPUT);
  gpio_config_t standbyConfig = {
    .pin_bit_mask = static_cast<uint64_t>(0x1 << static_cast<uint64_t>(standbyPin)),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
	ESP_ERROR_CHECK(gpio_config(&standbyConfig));
  gpio_set_level(standbyPin, true);

  configureMotor(&leftMotorProperties);
  configureMotor(&rightMotorProperties);

  xTaskCreatePinnedToCore(
        motorServiceTask,      // function that should be called
        "motorServiceTask",    // name of the task (for debugging)
        10000,               // stack size (bytes)
        NULL,               // parameter to pass
        motorServiceConfig.taskPriority,                  // task priority
        &motorServiceConfig.taskHandle,               // task handle
        motorServiceConfig.taskCore          // core you want to run the task on (0 or 1)
    );
  
  ESP_LOGI("Controller", "Konfiguracja silnikow zakonczona.");
}

/*
There's an entire ready-to-go example of PWM PID BDC motor control on Espressif GitHub.
However, it doesn't properly support motor driver used in this project (TB6612FNG).
A new version of standalone MCPWM could be used, but it'd be time consuming.
Since time is of the essence, it's been decided to stick with deprecated version.
*/
void motorServiceTask(void *args){
  float lowerThreshold;
  float upperThreshold;
  bool leftInRange;
  bool rightInRange;
  // PWM start
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, leftMotorProperties.pwmPin));
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, rightMotorProperties.pwmPin));

  // motor PID
  pid_ctrl_config_t pidConf = {
      .init_param = leftMotorPid.params,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConf, &leftMotorProperties.pidCtrl));
  
  pidConf = {
      .init_param = rightMotorPid.params,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConf, &rightMotorProperties.pidCtrl));

  pidConf = {
      .init_param = distancePidConf.params,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConf, &pidHandle));

  // motor loop
  const esp_timer_create_args_t loopTimerArgs = {
      .callback = motorLoop,
      .arg = &pidHandle,
      .name = "motorLoop"
  };
  ESP_ERROR_CHECK(esp_timer_create(&loopTimerArgs, &loopTimer));
  
  while(true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(executingTask && (pidOutput.size() == 20)){
      auto meanAndStdDev = calculateMeanAndSampleStandardDeviation(pidOutput);
      measurements.mean = meanAndStdDev.first;
      measurements.standardDeviation = meanAndStdDev.second;
      lowerThreshold = meanAndStdDev.first - (outputErrorTolerance * 0.01) * meanAndStdDev.second;
      upperThreshold = meanAndStdDev.first + (outputErrorTolerance * 0.01) * meanAndStdDev.second;
      measurements.lowerThreshold = lowerThreshold;
      measurements.upperThreshold = upperThreshold;
      leftInRange = std::all_of(pidOutputLeft.begin(), pidOutputLeft.end(), [lowerThreshold, upperThreshold](int leftOutput) {
        return leftOutput >= lowerThreshold && leftOutput <= upperThreshold;
      });
      rightInRange = std::all_of(pidOutputRight.begin(), pidOutputRight.end(), [lowerThreshold, upperThreshold](int rightOutput) {
        return rightOutput >= lowerThreshold && rightOutput <= upperThreshold;
      });

      ESP_LOGI("check", "up thresh: %f", upperThreshold);
      ESP_LOGI("check", "low thresh: %f", lowerThreshold);
      ESP_LOGI("check", "leftrange: %d, rightrange: %d", leftInRange, rightInRange);
      ESP_LOGI("check", "stddev: %f", meanAndStdDev.second);
      if(leftInRange && rightInRange){
        stopLoop();
      }
    }
  }
}

void configureMotor(motorProperties *args){
  motorProperties *motor = (motorProperties *)args;
  
  gpio_set_direction(motor->pwmPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(motor->motorPolarization_IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(motor->motorPolarization_IN2, GPIO_MODE_OUTPUT);
  
  gpio_set_direction(motor->encoderLeftPin, GPIO_MODE_INPUT);
  gpio_set_direction(motor->encoderRightPin, GPIO_MODE_INPUT);

  // PWM config
  mcpwm_config_t pwmConfig = {
    pwmConfig.frequency = 25000,
    pwmConfig.cmpr_a = 0,
    pwmConfig.cmpr_b = 0,
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0,
    pwmConfig.counter_mode = MCPWM_UP_COUNTER,
  };
	ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig));
  motor->mcpwmConfig = pwmConfig;

  // polarization config
  gpio_config_t gpioConf;
  gpioConf.intr_type = GPIO_INTR_DISABLE;
  gpioConf.mode = GPIO_MODE_OUTPUT;
  gpioConf.pin_bit_mask = 0x1<<motor->motorPolarization_IN1 | 0x1<<motor->motorPolarization_IN2;
  gpioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&gpioConf));

  // PCNT
  pcnt_unit_config_t pcntUnitConfig = {
      .low_limit = pcntLowLimit,
      .high_limit = pcntHighLimit,
      .flags = {
          .accum_count = true
      }
  };
  pcnt_unit_handle_t pcntUnit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&pcntUnitConfig, &pcntUnit));
  pcnt_glitch_filter_config_t pcntFilterConfig = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcntUnit, &pcntFilterConfig));
  pcnt_chan_config_t channelAConfig = {
      .edge_gpio_num = motor->encoderLeftPin,
      .level_gpio_num = motor->encoderRightPin,
  };
  pcnt_channel_handle_t pcntChannelA = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnit, &channelAConfig, &pcntChannelA));
  pcnt_chan_config_t channelBConfig = {
      .edge_gpio_num = motor->encoderRightPin,
      .level_gpio_num = motor->encoderLeftPin,
  };
  pcnt_channel_handle_t pcntChannelB = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnit, &channelBConfig, &pcntChannelB));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntChannelA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntChannelA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcntChannelB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcntChannelB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnit, pcntHighLimit));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcntUnit, pcntLowLimit));
  ESP_ERROR_CHECK(pcnt_unit_enable(pcntUnit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcntUnit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcntUnit));
  motor->pcntUnit = pcntUnit;
}

void driveMotor(motorProperties *args){
  motorProperties *motor = (motorProperties *)args;
  ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, motor->mcpwmGenerator, motor->motorSpeed));
  gpio_set_level(motor->motorPolarization_IN1, motor->motorDir); 
  gpio_set_level(motor->motorPolarization_IN2, !(motor->motorDir));
}

std::pair<float, float> calculateMeanAndSampleStandardDeviation(const std::deque<int>& data){
  if (data.size() <= 1)
      return {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()};

  float mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

  float sumSquaredDifferences = std::accumulate(data.begin(), data.end(), 0.0, [mean](float acc, float value) {
      return acc + std::pow(value - mean, 2);
  });

  float sampleStdDeviation = std::sqrt(sumSquaredDifferences / (data.size() - 1));

  return {mean, sampleStdDeviation};
}