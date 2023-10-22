#include <Controller.hpp>
#include <Config.hpp>
#include <Connectivity.hpp>
#include <MessageProcessor.hpp>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" void app_main(void){
  ESP_LOGI("Setup", "Rozpoczynanie konfiguracji.");
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // enable printf without \n
  setvbuf(stdout, NULL, _IONBF, 0);

  // setup motors and interrupts
  setupMotors();

  // start wifi service
  connectWifi();

  // create udp packet receiver task
  xTaskCreatePinnedToCore(
    receiveUDPPacket,      // Function that should be called
    "receiveUDPPacket",    // Name of the task (for debugging)
    10000,               // Stack size (bytes)
    NULL,               // Parameter to pass
    receiveUDPPacketPriority,                  // Task priority
    NULL,               // Task handle
    receiveUDPPacketCore          // Core you want to run the task on (0 or 1)
  );

  while(!connected) xTaskDelayUntil(&xLastWakeTime, 20);;
  ESP_LOGI("Setup","Konfiguracja zakonczona. Rozpoczynanie dzialania programu.");

  while(true){
    xTaskDelayUntil(&xLastWakeTime, 20);
  }
}