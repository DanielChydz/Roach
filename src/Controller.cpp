#include "Config.hpp"
#include <esp_log.h>
#include <esp_timer.h>

bool executingTask = false;

void motorServiceTask(void *args);
void driveMotor(motorProperties *args);
void configureMotor(motorProperties *args);

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

    if(leftMotorProperties.distance > 0 && rightMotorProperties.distance > 0){
      float error = distancePidConf.setPoint * pulsesPerRevolution - drivenWheelRevolutions * pulsesPerRevolution;
    }
    
    // set the new speed
    //pid_compute(pidCtrl, error, &leftMotorSpeed);
    //leftMotorSpeed /= 140;
    //if(leftMotorSpeed > 100) leftMotorSpeed = 100;
    //leftMotorSpeed = speedSetPoint;
    driveMotor(&leftMotorProperties);
    driveMotor(&rightMotorProperties);
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
}

/*
There's an entire ready-to-go example of PWM PID BDC motor control on Espressif GitHub.
However, it doesn't properly support motor driver used in this project (TB6612FNG).
A new version of standalone MCPWM could be used, but it'd be time consuming.
Since time is of the essence, it's been decided to stick with deprecated version.
*/
void motorServiceTask(void *args){
  // PID
  pid_ctrl_parameter_t pidRuntimeParams = {
      .kp = distancePidConf.pidKp,
      .ki = distancePidConf.pidKi,
      .kd = distancePidConf.pidKd,
      .max_output = distancePidConf.maxOutput,
      .min_output = distancePidConf.minOutput,
      .max_integral = distancePidConf.maxIntegral,
      .min_integral = distancePidConf.minIntegral,
      .cal_type = PID_CAL_TYPE_POSITIONAL,
  };
  pid_ctrl_block_handle_t pidCtrl = NULL;
  pid_ctrl_config_t pidConfig = {
      .init_param = pidRuntimeParams,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConfig, &pidCtrl));

  // PWM start
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, leftMotorProperties.pwmPin));
  ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, rightMotorProperties.pwmPin));

  // motor PID
  pidRuntimeParams = {
      .kp = leftMotorPid.pidKp,
      .ki = leftMotorPid.pidKi,
      .kd = leftMotorPid.pidKd,
      .max_output = leftMotorPid.maxOutput,
      .min_output = leftMotorPid.minOutput,
      .max_integral = leftMotorPid.maxIntegral,
      .min_integral = leftMotorPid.minIntegral,
      .cal_type = PID_CAL_TYPE_POSITIONAL,
  };
  pidCtrl = NULL;
  pidConfig = {
      .init_param = pidRuntimeParams,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConfig, &pidCtrl));
  leftMotorProperties.pidCtrl = pidCtrl;
  
  pidRuntimeParams = {
      .kp = rightMotorPid.pidKp,
      .ki = rightMotorPid.pidKi,
      .kd = rightMotorPid.pidKd,
      .max_output = rightMotorPid.maxOutput,
      .min_output = rightMotorPid.minOutput,
      .max_integral = rightMotorPid.maxIntegral,
      .min_integral = rightMotorPid.minIntegral,
      .cal_type = PID_CAL_TYPE_POSITIONAL,
  };
  pidCtrl = NULL;
  pidConfig = {
      .init_param = pidRuntimeParams,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pidConfig, &pidCtrl));
  rightMotorProperties.pidCtrl = pidCtrl;

  // motor loop
  const esp_timer_create_args_t loopTimerArgs = {
      .callback = motorLoop,
      .arg = &pidCtrl,
      .name = "motorLoop"
  };
  esp_timer_handle_t loopTimer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&loopTimerArgs, &loopTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(loopTimer, distancePidConf.loopPeriod * 1000));
  
  while(true) {
      vTaskDelay(pdMS_TO_TICKS(500));
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

void brake(){
  // TODO
}

void driveMotor(motorProperties *args){
  motorProperties *motor = (motorProperties *)args;
  ESP_ERROR_CHECK(mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, motor->mcpwmGenerator, motor->motorSpeed));
  gpio_set_level(motor->motorPolarization_IN1, motor->motorDir); 
  gpio_set_level(motor->motorPolarization_IN2, !(motor->motorDir));
}