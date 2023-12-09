#include "Config.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/mcpwm.h>
#include <../bdc_motor/include/bdc_motor.h>
#include <../pid_ctrl/include/pid_ctrl.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <freertos/queue.h>
#include <esp_timer.h>

TaskHandle_t xMotorServiceHandle;
motorProperties leftMotorProperties, rightMotorProperties;
float pidSetPoint;
bool unit;

void encLeft_leftPin();
void encLeft_rightPin();
void motorServiceTask(void *args);
void setMotorRotationDir(gpio_num_t firstPin, gpio_num_t secondPin, bool dir);
void driveVehicle();

static void motorLoop(void *args)
{
    static int lastPulseCountLeftMotor = 0;
    static int lastPulseCountRightMotor = 0;
    static int drivenWheelRevolutions = 0;
    static int drivenWheelDistance = 0;
    pcnt_unit_get_count(leftMotorProperties.pcntUnit, &leftMotorProperties.pulses);
    pcnt_unit_get_count(rightMotorProperties.pcntUnit, &rightMotorProperties.pulses);
    leftMotorProperties.loopPulses = leftMotorProperties.pulses - lastPulseCountLeftMotor;
    rightMotorProperties.loopPulses = rightMotorProperties.pulses - lastPulseCountRightMotor;

    // calculate the speed error
    if(unit){ // wheel revolutions
      drivenWheelRevolutions = (leftMotorProperties.pulses + rightMotorProperties.pulses) / 2800; // (1/2)/1400=1/2800
      float error = pidSetPoint * pulsesPerRevolution - drivenWheelRevolutions * pulsesPerRevolution;
    } else { // distance, cm
      drivenWheelRevolutions = (leftMotorProperties.pulses + rightMotorProperties.pulses) / 2800; // (1/2)/1400=1/2800
      float error = pidSetPoint - drivenWheelRevolutions * pulsesPerRevolution;
    }
    float error = pidSetPoint - ((leftMotorProperties.pulses + rightMotorProperties.pulses)/2);

    // set the new speed
    //pid_compute(pidCtrl, error, &leftMotorSpeed);
    //leftMotorSpeed /= 140;
    //if(leftMotorSpeed > 100) leftMotorSpeed = 100;
    //leftMotorSpeed = speedSetPoint;
    driveVehicle();
}

void startMotorService() {
  ESP_LOGI("Controller", "Konfiguracja silnikow.");

  gpio_set_direction(standbyPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotorProperties.pwmPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotorProperties.motorPolarization_IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotorProperties.motorPolarization_IN2, GPIO_MODE_OUTPUT);

  gpio_set_direction(rightMotorProperties.pwmPin, GPIO_MODE_OUTPUT);
  gpio_set_direction(rightMotorProperties.motorPolarization_IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(rightMotorProperties.motorPolarization_IN2, GPIO_MODE_OUTPUT);

  leftMotorProperties = {
    .pwmPin = GPIO_NUM_4,
    .motorPolarization_IN1 = GPIO_NUM_16,
    .motorPolarization_IN2 = GPIO_NUM_17,
    .encoderLeftPin = GPIO_NUM_18,
    .encoderRightPin = GPIO_NUM_19,
  };

  rightMotorProperties = {
    .pwmPin = GPIO_NUM_25,
    .motorPolarization_IN1 = GPIO_NUM_26,
    .motorPolarization_IN2 = GPIO_NUM_27,
    .encoderLeftPin = GPIO_NUM_32,
    .encoderRightPin = GPIO_NUM_33,
  };

  gpio_config_t standbyConfig = {
    .pin_bit_mask = 0x1<<standbyPin,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
	ESP_ERROR_CHECK(gpio_config(&standbyConfig));
  gpio_set_level(standbyPin, true);

  gpio_set_direction(rightMotorProperties.encoderLeftPin, GPIO_MODE_INPUT);
  gpio_set_direction(rightMotorProperties.encoderRightPin, GPIO_MODE_INPUT);

  xTaskCreatePinnedToCore(
        motorServiceTask,      // function that should be called
        "motorServiceTask",    // name of the task (for debugging)
        10000,               // stack size (bytes)
        NULL,               // parameter to pass
        motorServiceConfig.taskPriority,                  // task priority
        &motorServiceConfig.taskHandle,               // task handle
        motorServiceConfig.taskCore          // core you want to run the task on (0 or 1)
    );
}

/*
There's an entire ready-to-go example of PWM PID BDC motor control on Espressif GitHub.
However, it doesn't properly support motor driver used in this project (TB6612FNG).
A new version of standalone MCPWM could be used, but it'd be time consuming.
Since time is of the essence, it's been decided to stick with deprecated version.
*/
void motorServiceTask(void *args){
  motorProperties *motorConfigArg = (motorProperties *)args;
  
  // PWM config
  mcpwm_config_t pwmConfig = {
    pwmConfig.frequency = 25000,
    pwmConfig.cmpr_a = 0,
    pwmConfig.cmpr_b = 0,
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0,
    pwmConfig.counter_mode = MCPWM_UP_COUNTER,
  };
	ESP_ERROR_CHECK(mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig));
  motorConfigArg->mcpwmConfig = pwmConfig;

  // polarization config
  gpio_config_t gpioConf;
  gpioConf.intr_type = GPIO_INTR_DISABLE;
  gpioConf.mode = GPIO_MODE_OUTPUT;
  gpioConf.pin_bit_mask = 0x1<<motorConfigArg->motorPolarization_IN1 | 0x1<<motorConfigArg->motorPolarization_IN2;
  gpioConf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpioConf.pull_up_en = GPIO_PULLUP_ENABLE;
  ESP_ERROR_CHECK(gpio_config(&gpioConf));

  // PWM start
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, motorConfigArg->pwmPin));
  setMotorRotationDir(motorConfigArg->motorPolarization_IN1, motorConfigArg->motorPolarization_IN2, 0);

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
      .edge_gpio_num = motorConfigArg->encoderLeftPin,
      .level_gpio_num = motorConfigArg->encoderRightPin,
  };
  pcnt_channel_handle_t pcntChannelA = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcntUnit, &channelAConfig, &pcntChannelA));
  pcnt_chan_config_t channelBConfig = {
      .edge_gpio_num = motorConfigArg->encoderRightPin,
      .level_gpio_num = motorConfigArg->encoderLeftPin,
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
  leftMotorProperties.pcntUnit = pcntUnit;

  // PID
  // pid_ctrl_parameter_t pidRuntimeParams = {
  //     .kp = pidKp,
  //     .ki = pidKi,
  //     .kd = pidKd,
  //     .max_output   = 14000,
  //     .min_output   = 0,
  //     .max_integral = 14000,
  //     .min_integral = 0,
  //     .cal_type = PID_CAL_TYPE_POSITIONAL,
  // };
  // pid_ctrl_block_handle_t pidCtrl = NULL;
  // pid_ctrl_config_t pidConfig = {
  //     .init_param = pidRuntimeParams,
  // };
  // ESP_ERROR_CHECK(pid_new_control_block(&pidConfig, &pidCtrl));
  // leftMotorProperties.pidCtrl = pidCtrl;

  // motor loop
  const esp_timer_create_args_t leftMotorTimerArgs = {
      .callback = motorLoop,
      .arg = &leftMotorProperties,
      .name = "leftMotorLoop"
  };
  esp_timer_handle_t leftMotorTimer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&leftMotorTimerArgs, &leftMotorTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(leftMotorTimer, loopPeriod * 1000));
  
  while(true) {
      vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// First direction pin, second direction pin, value 0/1.
void setMotorRotationDir(gpio_num_t firstPin, gpio_num_t secondPin, bool dir){
  gpio_set_level(firstPin, dir); 
  gpio_set_level(secondPin, !dir);
}

// PWM pin, speed value 0-255
void setMotorSpeed(gpio_num_t motor, int speed) {
  gpio_set_level(motor, speed); // CHANGE ASAP
}

void brake(){
  // TODO
}

void driveVehicle(motorProperties *motorConfigArg){
  ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_GEN_A, motorConfigArg->motorSpeed));
  setMotorRotationDir(motorConfigArg->motorPolarization_IN1, motorConfigArg->motorPolarization_IN2, motorConfigArg->motorDir);
}

/*
Function inteded mainly for debugging purposes.
Wheel: 0=both wheels, 1=left, 2=right
Times: how many times is/are the wheel(s) supposed to rotate, positive non-zero fractions allowed
Speed: speed of rotation (0-255)
Directions:
For one wheel: 0=left, 1=right
For both wheels: 0=left/right, 1=right/left, 2=left/left, 3=right/right
*/
void rotateWheels(int wheel, float times, int speed, int direction){
  // switch(wheel){
  //   case 0:
  //     switch(direction){
  //       case 0:
  //         setMotorsRotationDir(direction, !direction);
  //         break;
  //       case 1:
  //         setMotorsRotationDir(!direction, direction);
  //         break;
  //       case 2:
  //         setMotorsRotationDir(direction, direction);
  //         break;
  //       case 3:
  //         setMotorsRotationDir(!direction, !direction);
  //         break;
  //     }
  //     setMotorsSpeed(speed, speed);
  //     requiredPulses = pulsesPerRotation * times;
  //     break;
  //   case 1:
  //     setMotorRotationDir(leftMotorProperties.motorPolarization_IN1, leftMotorProperties.motorPolarization_IN2, direction);
  //     setMotorSpeed(leftMotorProperties.pwmPin, speed);
  //     requiredPulses = pulsesPerRotation * times;
  //     break;
  //   case 2:
  //     setMotorRotationDir(rightMotorProperties.motorPolarization_IN1, rightMotorProperties.motorPolarization_IN2, direction);
  //     setMotorSpeed(rightMotorProperties.pwmPin, speed);
  //     requiredPulses = pulsesPerRotation * times;
  //     break;
  // }
}