#include <Arduino.h>
#include <Config.hpp>

uint8_t speed = 0;
uint8_t firstMotorSpeed = 0;
uint8_t secondMotorSpeed = 0;
uint8_t command = 0;
float distance = 0;
bool firstMotorDir = 0;
bool secondMotorDir = 0;

int leftEncoder = 0;
int rightEncoder = 0;

bool leftEncoder_leftPin;
bool leftEncoder_rightPin;
bool rightEncoder_leftPin;
bool rightEncoder_rightPin;

short requiredPulses = 0;
short currentPulses = 0;

void encLeft_leftPin();
void encLeft_rightPin();
void attachInterrupts(void *param);

void setStandby(int pin, bool mode){
  digitalWrite(pin, mode);
}

void setupMotors() {
  Serial.println("Konfiguracja silnikow.");

  pinMode(leftMotor.pin, OUTPUT);
  pinMode(leftMotor.standby, OUTPUT);
  pinMode(leftMotor.motorPolarization_IN1, OUTPUT);
  pinMode(leftMotor.motorPolarization_IN2, OUTPUT);

  pinMode(rightMotor.pin, OUTPUT);
  pinMode(rightMotor.motorPolarization_IN1, OUTPUT);
  pinMode(rightMotor.motorPolarization_IN2, OUTPUT);

  setStandby(leftMotor.standby, HIGH);

  pinMode(leftMotor.encoderLeftPin, INPUT);
  pinMode(leftMotor.encoderRightPin, INPUT);

  xTaskCreatePinnedToCore(
    &attachInterrupts,      // Function that should be called
    "attachInterrupts",    // Name of the task (for debugging)
    10000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    attachInterruptsPriority,                  // Task priority
    NULL,               // Task handle
    attachInterruptsCore          // Core you want to run the task on (0 or 1)
  );
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
  setMotorSpeed(leftMotor.pin, first);
  setMotorSpeed(rightMotor.pin, second);
}

// First motor rotation direction, second motor rotation direction, values 0/1.
void setMotorsRotationDir(int first, int second){
  setMotorRotationDir(leftMotor.motorPolarization_IN1, leftMotor.motorPolarization_IN2, first);
  setMotorRotationDir(rightMotor.motorPolarization_IN1, rightMotor.motorPolarization_IN2, second);
}

void brake(){
  setMotorsSpeed(1, 1);
  digitalWrite(leftMotor.motorPolarization_IN1, 0);
  digitalWrite(leftMotor.motorPolarization_IN2, 0);
  digitalWrite(rightMotor.motorPolarization_IN1, 0);
  digitalWrite(rightMotor.motorPolarization_IN2, 0);
}

void driveVehicle(){
  // T.B.C.
}

void attachInterrupts(void *param){
  attachInterrupt(leftMotor.encoderLeftPin, encLeft_leftPin, CHANGE);
  attachInterrupt(leftMotor.encoderRightPin, encLeft_rightPin, CHANGE);

  attachInterrupt(rightMotor.encoderLeftPin, encLeft_leftPin, CHANGE);
  attachInterrupt(rightMotor.encoderRightPin, encLeft_rightPin, CHANGE);
  vTaskDelete(NULL); // would be good to move it outside the task in final version
}

void encLeft_leftPin(){
  leftEncoder_leftPin = digitalRead(leftMotor.encoderLeftPin);
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
  leftEncoder_rightPin = digitalRead(leftMotor.encoderRightPin);
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

void encRight_leftPin(){rightEncoder_leftPin = digitalRead(rightMotor.encoderLeftPin);}
void encRight_rightPin(){rightEncoder_rightPin = digitalRead(rightMotor.encoderRightPin);}

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