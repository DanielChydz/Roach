#include <driver/gpio.h>

extern uint8_t speed, firstMotorSpeed, secondMotorSpeed, command;

void setupMotors();

void setMotorRotationDir(gpio_num_t firstPin, gpio_num_t secondPin, bool dir);
void setMotorSpeed(gpio_num_t motor, int speed);
void setStandby(gpio_num_t pin, bool mode);
void driveVehicle();
void brake();

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsSpeed(bool first, bool second);

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsRotationDir(bool first, bool second);

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