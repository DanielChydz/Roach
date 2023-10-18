#include <Arduino.h>
#include <Regulator.hpp>
#include <string>
#include <Config.hpp>
#include <Arduino.h>
#include <Connectivity.hpp>
#include <MessageProcessor.hpp>
#include <Credentials.hpp>

int testt = 1;

void test(void *param);

// setup software
void setup() {
  // setup serial interface
  Serial.begin(9600);
  Serial.println("\nRozpoczynanie konfiguracji.");

  // pulling credentials from Credentials.hpp
  ConnectivityData.ssid = ssidCredential;
  ConnectivityData.password = passwordCredential;

  setupMotors();

  xTaskCreatePinnedToCore(
    test,      // Function that should be called
    "test1",    // Name of the task (for debugging)
    1000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    1,                  // Task priority
    NULL,               // Task handle
    0          // Core you want to run the task on (0 or 1)
  );

  //runMaintainWifiConnectionRoutine();
  //udp.begin(ConnectivityData.udpBeginPort);

  Serial.println("\nKonfiguracja zakonczona. Rozpoczynanie dzialania programu.");
}

void loop() {
  // if(connected){
  //   runMaintainWifiConnectionRoutine();

  //   runReceiveUDPRoutine();
  // }

  Serial.println(leftEncoder);
  delay(1000);
}

void test(void *param){
  while(true){
    rotateWheels(1, 5, 64, 0);
    delay(10000);
  }
}