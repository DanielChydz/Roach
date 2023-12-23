#ifndef ROACH_CONTR_H
#define ROACH_CONTR_H
#include "Connectivity.hpp"
#include "../components/pid_ctrl/include/pid_ctrl.h"
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pulse_cnt.h>

// motor config struct
struct motorProperties{
    const gpio_num_t pwmPin; // PWM
    const gpio_num_t motorPolarization_IN1; // polarization 1
    const gpio_num_t motorPolarization_IN2; // polarization 2
    const gpio_num_t encoderLeftPin;
    const gpio_num_t encoderRightPin;
    pcnt_unit_handle_t pcntUnit;
    mcpwm_config_t mcpwmConfig;
    pid_ctrl_block_handle_t pidCtrl;
    int16_t loopPulses;
    int32_t distance;
    int pulses;
    float motorSpeed;
    bool motorDir;
    mcpwm_generator_t mcpwmGenerator;
};

struct measurementData{
    int mean;
    int standardDeviation;
    int upperThreshold;
    int lowerThreshold;
};

extern measurementData measurements;
extern motorProperties leftMotorProperties, rightMotorProperties;
extern bool executingTask;

void startMotorService();
void startLoop();
void stopLoop();
#endif