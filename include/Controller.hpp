#ifndef ROACH_CONTR_H
#define ROACH_CONTR_H
#include <driver/gpio.h>
#include <driver/mcpwm.h>
#include <driver/pulse_cnt.h>

// motor config struct
typedef struct {
    gpio_num_t pwmPin; // PWM
    gpio_num_t motorPolarization_IN1; // polarization 1
    gpio_num_t motorPolarization_IN2; // polarization 2
    gpio_num_t encoderLeftPin;
    gpio_num_t encoderRightPin;
    pcnt_unit_handle_t pcntUnit;
    mcpwm_config_t mcpwmConfig;
    int16_t loopPulses;
    int pulses;
    uint16_t motorSpeed;
    bool motorDir;
} motorProperties;

extern motorProperties leftMotorProperties, rightMotorProperties;
extern float pidSetPoint;
extern bool unit;

void startMotorService();
void setMotorRotationDir(gpio_num_t firstPin, gpio_num_t secondPin, bool dir);
void setMotorSpeed(gpio_num_t motor, int speed);
void setStandby(gpio_num_t pin, bool mode);
void driveVehicle();
void brake();

/*
* Function inteded mainly for debugging purposes.
* Wheel: 0=both wheels, 1=left, 2=right
* Times: how many times is/are the wheel(s) supposed to rotate, positive non-zero fractions allowed
* Speed: speed of rotation (0-255)
* Directions:
* For one wheel: 0=left, 1=right
* For both wheels: 0=left/right, 1=right/left, 2=left/left, 3=right/right
*/
void rotateWheels(int wheel, float times, int speed, int direction);

#endif