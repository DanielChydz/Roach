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

  // setup motors and interrupts
  setupMotors();

  // start wifi connection maintenance task
  xTaskCreatePinnedToCore(
    &maintainWifiConnection,      // Function that should be called
    "maintainWifiConnection",    // Name of the task (for debugging)
    1000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    maintainWifiConnectionPriority,                  // Task priority
    NULL,               // Task handle
    maintainWifiConnectionCore          // Core you want to run the task on (0 or 1)
  );

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