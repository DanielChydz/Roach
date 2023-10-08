extern int angle, axis, firstMotorSpeed, secondMotorSpeed, speed, radius, leftEncoder;
extern double deltaCurrent, deltaLastRun;

void setupMotors();

void setMotorRotationDir(int firstPin, int secondPin, bool dir);
void setMotorSpeed(int motor, int speed);
void setStandby(int pin, bool mode);
void driveVehicle();
void brake();

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsSpeed(int first, int second);

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsRotationDir(int first, int second);

/*
Function inteded mainly for debugging purposes.
Wheel: 0=both wheels, 1=left, 2=right
Times: how many times is/are the wheel(s) supposed to rotate, positive non-zero fractions allowed
Speed: speed of rotation (0-255)
Directions:
For one wheel: 0=left, 1=right
For both wheels: 0=left/right, 1=right/left, 2=left/left, 3=right/right
*/
void rotateWheels(int wheel, float times, int speed, int direction);