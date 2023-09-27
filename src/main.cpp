#include <Arduino.h>
#include <Motors.hpp>
#include <AceRoutine.h>
#include <string>
#include <Config.hpp>
#include <Arduino.h>
#include <Connectivity.hpp>
#include <MessageProcessor.hpp>
#include <Credentials.hpp>

// setup software
void setup() {
  // setup serial interface
  Serial.begin(9600);
  Serial.println("\nRozpoczynanie konfiguracji.");

  // pulling credentials from Credentials.hpp
  ConnectivityData.ssid = ssidCredential;
  ConnectivityData.password =passwordCredential;

  Serial.println(ConnectivityData.ssid);
  Serial.println(ConnectivityData.password);

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

  Serial.println("Test.");

  delay(200);
}