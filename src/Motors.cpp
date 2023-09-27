/*          1  M1                                   3  M2
          |-------|                               |-------|
         1|       |               F2              |       |0
          |       |-------------------------------|       |
         0|       |                               |       |1
          |-------|                               |-------|
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
                                  |
             M3                   |                   M4
          |-------|                               |-------|
         1|       |               R4              |       |0
          |       |-------------------------------|       |
         0|       |                               |       |1
          |-------|                               |-------|
*/

#include <Arduino.h>
#include <Config.hpp>
#include <AceRoutine.h>

const double pi = 3.14159265;

int angle = 0;
int axis = 0;
int speed = 0;
int radius = 0;
int firstMotorSpeed = 0;
int secondMotorSpeed = 0;
int firstMotorDir = 0;
int secondMotorDir = 0;

void setStandby(int pin, bool mode){
  digitalWrite(pin, mode);
}

void setupMotors() {
  Serial.println("Konfiguracja silnikow.");

  pinMode(firstMotor.pin, OUTPUT); 
  pinMode(firstMotor.standby, OUTPUT);
  pinMode(firstMotor.motorPolarization_IN1, OUTPUT);
  pinMode(firstMotor.motorPolarization_IN2, OUTPUT);

  pinMode(secondMotor.pin, OUTPUT);
  pinMode(secondMotor.motorPolarization_IN1, OUTPUT);
  pinMode(secondMotor.motorPolarization_IN2, OUTPUT);

  setStandby(firstMotor.standby, HIGH);
  }

// pierwszy pin sterujący, drugi pin sterujący, kierunek 0/1
void setMotorRotationDir(int firstPin, int secondPin, bool dir){
  digitalWrite(firstPin, dir); 
  digitalWrite(secondPin, !dir);
}

// pin PWM, predkosc 0-255
void setMotorSpeed(int motor, int speed) {
  analogWrite(motor, speed);
}

// predkosc pierwszego silnika, predkosc drugiego silnika, 0-255
void setMotorsSpeed(int first, int second){
  setMotorSpeed(firstMotor.pin, first);
  setMotorSpeed(secondMotor.pin, second);
}

// kierunek obrotu pierwszego silnika, kierunek obrotu drugiego silnika, 0/1
void setMotorsRotationDir(int first, int second){
  setMotorRotationDir(firstMotor.motorPolarization_IN1, firstMotor.motorPolarization_IN2, first);
  setMotorRotationDir(secondMotor.motorPolarization_IN1, secondMotor.motorPolarization_IN2, second);
}

void brake(){
  setMotorsSpeed(1, 1);
  digitalWrite(firstMotor.motorPolarization_IN1, 0);
  digitalWrite(firstMotor.motorPolarization_IN2, 0);
  digitalWrite(secondMotor.motorPolarization_IN1, 0);
  digitalWrite(secondMotor.motorPolarization_IN2, 0);
}

void driveVehicle(){
  if(speed == 0 && radius == 0) {
    brake();
  } else {
    if(angle < 180){
      setMotorRotationDir(firstMotor.motorPolarization_IN1, firstMotor.motorPolarization_IN2, 0);
    } else {
      setMotorRotationDir(firstMotor.motorPolarization_IN1, firstMotor.motorPolarization_IN2, 1);
    }
    if(axis < 180){
      setMotorRotationDir(secondMotor.motorPolarization_IN1, secondMotor.motorPolarization_IN2, 0);
    } else {
      setMotorRotationDir(secondMotor.motorPolarization_IN1, secondMotor.motorPolarization_IN2, 1);
    }
    speed = map(speed, 0, 100, 0, 255);
    radius = map(radius, 0, 100, 0, 255);
    setMotorsSpeed(speed, radius);
  }
}