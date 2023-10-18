#include <Arduino.h>
#include <Config.hpp>

const double pi = 3.14159265;

int angle = 0;
int axis = 0;
int speed = 0;
int radius = 0;
int firstMotorSpeed = 0;
int secondMotorSpeed = 0;
int firstMotorDir = 0;
int secondMotorDir = 0;

int leftEncoderCount = 0;
int rightEncoderCount = 0;

int leftEncoder = 0;
int rightEncoder = 0;

int leftEncoder_leftPin;
int leftEncoder_rightPin;
int rightEncoder_leftPin;
int rightEncoder_rightPin;

int requiredPulses = 0;
int currentPulses = 0;

void encLeft_leftPin();
void encLeft_rightPin();

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

  pinMode(firstMotor.encoderLeftPin, INPUT);
  pinMode(firstMotor.encoderRightPin, INPUT);
  attachInterrupt(firstMotor.encoderLeftPin, encLeft_leftPin, CHANGE);
  attachInterrupt(firstMotor.encoderRightPin, encLeft_rightPin, CHANGE);

  // attachInterrupt(secondMotor.encoderLeftPin, encLeft_leftPin_High, FALLING);
  // attachInterrupt(secondMotor.encoderLeftPin, encLeft_leftPin_High, RISING);
  // attachInterrupt(secondMotor.encoderRightPin, encLeft_leftPin_High, FALLING);
  // attachInterrupt(secondMotor.encoderRightPin, encLeft_leftPin_High, RISING);
}

// First direction pin, second direction pin, value 0/1.
void setMotorRotationDir(int firstPin, int secondPin, bool dir){
  digitalWrite(firstPin, dir); 
  digitalWrite(secondPin, !dir);
}

// PWM pin, speed value 0-255
void setMotorSpeed(int motor, int speed) {
  analogWrite(motor, speed);
}

// First motor speed, second motor speed, values 0-255.
void setMotorsSpeed(int first, int second){
  setMotorSpeed(firstMotor.pin, first);
  setMotorSpeed(secondMotor.pin, second);
}

// First motor rotation direction, second motor rotation direction, values 0/1.
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

void encLeft_leftPin(){
  leftEncoder_leftPin = digitalRead(firstMotor.encoderLeftPin);
  if((!leftEncoder_leftPin && !leftEncoder_rightPin) || (leftEncoder_leftPin && leftEncoder_rightPin)){
    leftEncoder--;
  } else if((!leftEncoder_leftPin && leftEncoder_rightPin) || (leftEncoder_leftPin && !leftEncoder_rightPin)) {
    leftEncoder++;

    if(requiredPulses>0){
      currentPulses++;
      if(currentPulses>=requiredPulses){
        brake();
        requiredPulses=0;
        currentPulses=0;
      }
    }
  }
}

void encLeft_rightPin(){
  leftEncoder_rightPin = digitalRead(firstMotor.encoderRightPin);
  if(!leftEncoder_leftPin && leftEncoder_rightPin){
    leftEncoder--;
  } else if(leftEncoder_leftPin && !leftEncoder_rightPin) {
    leftEncoder++;

    if(requiredPulses>0){
      currentPulses++;
      if(currentPulses>=requiredPulses){
        brake();
        requiredPulses=0;
        currentPulses=0;
      }
    }
  }
}

void encRight_leftPin_Low(){leftEncoder_leftPin = 0;}
void encRight_leftPin_High(){leftEncoder_leftPin = 1;}
void encRight_rightPin_Low(){rightEncoder_rightPin = 0;}
void encRight_rightPin_High(){rightEncoder_rightPin = 1;}

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
      setMotorRotationDir(firstMotor.motorPolarization_IN1, firstMotor.motorPolarization_IN2, direction);
      setMotorSpeed(firstMotor.pin, speed);
      requiredPulses = pulsesPerRotation * times;
      break;
    case 2:
      setMotorRotationDir(secondMotor.motorPolarization_IN1, secondMotor.motorPolarization_IN2, direction);
      setMotorSpeed(secondMotor.pin, speed);
      requiredPulses = pulsesPerRotation * times;
      break;
  }
}