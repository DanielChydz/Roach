#include <Config.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/pulse_cnt.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <driver/mcpwm_timer.h>
#include <../bdc_motor/include/bdc_motor.h>
#include <../pid_ctrl/include/pid_ctrl.h>

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "esp_timer.h"

int32_t bothWheelsValue = 0;
int32_t leftWheelValue = 0;
int32_t rightWheelValue = 0;
uint8_t speedSetPoint = 0;
uint8_t leftMotorSpeed = 0;
uint8_t rightMotorSpeed = 0;
bool leftMotorDir = 0;
bool rightMotorDir = 0;
bool unit = 0;

int leftEncoder = 0;
int rightEncoder = 0;

bool leftEncoder_leftPin;
bool leftEncoder_rightPin;
bool rightEncoder_leftPin;
bool rightEncoder_rightPin;

short requiredPulses = 0;
short currentPulses = 0;

TaskHandle_t xMotorServiceHandle;

#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define BDC_ENCODER_PCNT_HIGH_LIMIT   1400
#define BDC_ENCODER_PCNT_LOW_LIMIT    -1400

#define BDC_PID_EXPECT_SPEED          400  // expected motor speed, in the pulses counted by the rotary encoder

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
} motor_control_context_t;

void encLeft_leftPin();
void encLeft_rightPin();
void motorServiceTask(void *pvParameters);

void setStandby(gpio_num_t pin, bool mode){
  gpio_set_level(pin, mode);
}

static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    ctx->report_pulses = real_pulses;

    // calculate the speed error
    float error = pidSetPoint - real_pulses;
    float new_speed = 0;

    // set the new speed
    pid_compute(pid_ctrl, error, &new_speed);
    bdc_motor_set_speed(motor, (uint32_t)new_speed);
}

void startMotorService() {
  xTaskCreatePinnedToCore(
        motorServiceTask,      // Function that should be called
        "motorServiceTask",    // Name of the task (for debugging)
        10000,               // Stack size (bytes)
        NULL,               // Parameter to pass
        motorServiceTaskPriority,                  // Task priority
        &xMotorServiceHandle,               // Task handle
        motorServiceTaskCore          // Core you want to run the task on (0 or 1)
    );
}

void motorServiceTask(void *pvParameters){
  ESP_LOGI("Controller", "Konfiguracja silnikow.");

  gpio_set_direction(leftMotor.pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotor.standby, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotor.motorPolarization_IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(leftMotor.motorPolarization_IN2, GPIO_MODE_OUTPUT);

  gpio_set_direction(rightMotor.pin, GPIO_MODE_OUTPUT);
  gpio_set_direction(rightMotor.motorPolarization_IN1, GPIO_MODE_OUTPUT);
  gpio_set_direction(rightMotor.motorPolarization_IN2, GPIO_MODE_OUTPUT);

  setStandby(leftMotor.standby, true);

  gpio_set_direction(leftMotor.encoderLeftPin, GPIO_MODE_INPUT);
  gpio_set_direction(leftMotor.encoderRightPin, GPIO_MODE_INPUT);

  // test
  static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
  };

  ESP_LOGI("MOTORTEST", "Create DC motor");
  bdc_motor_config_t motor_config = {
      .pwma_gpio_num = leftMotor.motorPolarization_IN1,
      .pwmb_gpio_num = leftMotor.motorPolarization_IN2,
      .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
  };
  bdc_motor_mcpwm_config_t mcpwm_config = {
      .group_id = 0,
      .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
  };
  bdc_motor_handle_t motor = NULL;
  ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
  motor_ctrl_ctx.motor = motor;

  ESP_LOGI("MOTORTEST", "Init pcnt driver to decode rotary signal");
  pcnt_unit_config_t unit_config = {
      .low_limit = pcntLowLimit,
      .high_limit = pcntHighLimit,
      .flags = {
          .accum_count = true // enable counter accumulation
      }
  };
  pcnt_unit_handle_t pcnt_unit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = leftMotor.encoderLeftPin,
      .level_gpio_num = leftMotor.encoderRightPin,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = leftMotor.encoderLeftPin,
      .level_gpio_num = leftMotor.encoderRightPin,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
  motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

  ESP_LOGI("MOTORTEST", "Create PID control block");
  pid_ctrl_parameter_t pid_runtime_param = {
      .kp = 0.6,
      .ki = 0.4,
      .kd = 0.2,
      .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
      .min_output   = 0,
      .max_integral = 1000,
      .min_integral = -1000,
      .cal_type = PID_CAL_TYPE_INCREMENTAL,
  };
  pid_ctrl_block_handle_t pid_ctrl = NULL;
  pid_ctrl_config_t pid_config = {
      .init_param = pid_runtime_param,
  };
  ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
  motor_ctrl_ctx.pid_ctrl = pid_ctrl;

  ESP_LOGI("MOTORTEST", "Create a timer to do PID calculation periodically");
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = pid_loop_cb,
      .arg = &motor_ctrl_ctx,
      .name = "pid_loop"
  };
  esp_timer_handle_t pid_loop_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

  ESP_LOGI("MOTORTEST", "Enable motor");
  ESP_ERROR_CHECK(bdc_motor_enable(motor));
  ESP_LOGI("MOTORTEST", "Forward motor");
  ESP_ERROR_CHECK(bdc_motor_forward(motor));

  ESP_LOGI("MOTORTEST", "Start motor speed loop");
  ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, pidLoopPeriod * 1000));

  while(true) {
      vTaskDelay(pdMS_TO_TICKS(100));
      // the following logging format is according to the requirement of serial-studio frame format
      // also see the dashboard config file `serial-studio-dashboard.json` for more information
      #if SERIAL_STUDIO_DEBUG
        printf("/*%d*/\r\n", motor_ctrl_ctx.report_pulses);
      #endif
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

// First motor speed, second motor speed, values 0-255.
void setMotorsSpeed(int first, int second){
  gpio_set_level(leftMotor.pin, first); // CHANGE ASAP
  gpio_set_level(rightMotor.pin, second); // CHANGE ASAP
}

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsRotationDir(bool first, bool second){
  setMotorRotationDir(leftMotor.motorPolarization_IN1, leftMotor.motorPolarization_IN2, first);
  setMotorRotationDir(rightMotor.motorPolarization_IN1, rightMotor.motorPolarization_IN2, second);
}

void brake(){
  setMotorsSpeed(1, 1);
  gpio_set_level(leftMotor.motorPolarization_IN1, 0);
  gpio_set_level(leftMotor.motorPolarization_IN2, 0);
  gpio_set_level(rightMotor.motorPolarization_IN1, 0);
  gpio_set_level(rightMotor.motorPolarization_IN2, 0);
}

void driveVehicle(){
  ESP_LOGI("Controller", "Oba kola: %d", (int)bothWheelsValue);
  ESP_LOGI("Controller", "Lewe kolo: %d", (int)leftWheelValue);
  ESP_LOGI("Controller", "Prawe kolo: %d", (int)rightWheelValue);
  ESP_LOGI("Controller", "Predkosc: %d", (int)speedSetPoint);
  ESP_LOGI("Controller", "Predkosc lewego kola: %d", (int)leftMotorSpeed);
  ESP_LOGI("Controller", "Predkosc prawego kola: %d", (int)rightMotorSpeed);
  ESP_LOGI("Controller", "Kierunek lewego kola: %d", (int)leftMotorDir);
  ESP_LOGI("Controller", "Kierunek prawego kola: %d", (int)rightMotorDir);
  ESP_LOGI("Controller", "Jednostka: %d", (int)unit);
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
  switch(wheel){
    case 0:
      switch(direction){
        case 0:
          setMotorsRotationDir(direction, !direction);
          break;
        case 1:
          setMotorsRotationDir(!direction, direction);
          break;
        case 2:
          setMotorsRotationDir(direction, direction);
          break;
        case 3:
          setMotorsRotationDir(!direction, !direction);
          break;
      }
      setMotorsSpeed(speed, speed);
      requiredPulses = pulsesPerRotation * times;
      break;
    case 1:
      setMotorRotationDir(leftMotor.motorPolarization_IN1, leftMotor.motorPolarization_IN2, direction);
      setMotorSpeed(leftMotor.pin, speed);
      requiredPulses = pulsesPerRotation * times;
      break;
    case 2:
      setMotorRotationDir(rightMotor.motorPolarization_IN1, rightMotor.motorPolarization_IN2, direction);
      setMotorSpeed(rightMotor.pin, speed);
      requiredPulses = pulsesPerRotation * times;
      break;
  }
}