#include <Arduino.h>
#include <Motors.hpp>
#include <AceRoutine.h>
#include <string>
#include <Config.hpp>
#include <Arduino.h>
#include <Connectivity.hpp>
#include <MessageProcessor.hpp>

// setup software
void setup() {
  // setup serial interface
  Serial.begin(9600);
  Serial.println("\nRozpoczynanie konfiguracji.");

  //setupMotors();
  //runMaintainWifiConnectionRoutine();
  //udp.begin(ConnectivityData.udpBeginPort);

  Serial.println("\nKonfiguracja zakonczona. Rozpoczynanie dzialania programu.");
}

void loop() {
  // if(connected){
  //   runMaintainWifiConnectionRoutine();

  //   runReceiveUDPRoutine();
  // }

  long unsigned duration, distance;

  pinMode(proximitySensor.trigPin, OUTPUT);
  pinMode(proximitySensor.echoPin, INPUT);

  digitalWrite(proximitySensor.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(proximitySensor.trigPin, LOW);

  duration = pulseIn(proximitySensor.echoPin, HIGH);

  distance = (duration/2) / 29.1;

  Serial.println(distance);

  delay(200);
}