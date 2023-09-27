extern int angle, axis, firstMotorSpeed, secondMotorSpeed, speed, radius;
extern double deltaCurrent, deltaLastRun;

void setupMotors();
void setMotorRotationDir(int firstPin, int secondPin, bool dir);
void setMotorSpeed(int motor, int speed);
void setStandby(int pin, bool mode);
void driveVehicle();
void brake();

// predkosc pierwszego silnika, predkosc drugiego silnika, 0-255
void setMotorsSpeed(int first, int second);

// kierunek obrotu pierwszego silnika, kierunek obrotu drugiego silnika, 0/1
void setMotorsRotationDir(int first, int second);