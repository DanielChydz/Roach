#include <Arduino.h>
#include <Controller.hpp>
#include <Config.hpp>
#include <Connectivity.hpp>
#include <MessageProcessor.hpp>

// setup software
void setup() {
  // setup serial interface
  Serial.begin(9600);
  Serial.println("\nRozpoczynanie konfiguracji.");

  // setup motors and interrupts
  setupMotors();

  // create tasks
  // create wifi connection maintenance task
  xTaskCreatePinnedToCore(
    maintainWifiConnection,      // Function that should be called
    "maintainWifiConnection",    // Name of the task (for debugging)
    10000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    maintainWifiConnectionPriority,                  // Task priority
    NULL,               // Task handle
    maintainWifiConnectionCore          // Core you want to run the task on (0 or 1)
  );
  xTaskCreatePinnedToCore(
    receiveUDPPacket,      // Function that should be called
    "receiveUDPPacket",    // Name of the task (for debugging)
    10000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    receiveUDPPacketPriority,                  // Task priority
    NULL,               // Task handle
    receiveUDPPacketCore          // Core you want to run the task on (0 or 1)
  );

  Serial.println("\nKonfiguracja zakonczona. Rozpoczynanie dzialania programu.");
}

void loop() {
  delay(1000);
}