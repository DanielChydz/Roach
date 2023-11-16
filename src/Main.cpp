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
  
  // setup wifi
  startWifiService();

  while(!connected) xTaskDelayUntil(&xLastWakeTime, 20);
  ESP_LOGI("Setup","Konfiguracja zakonczona. Rozpoczynanie dzialania programu.");

  while(true){
    xTaskDelayUntil(&xLastWakeTime, 20);
  }
}