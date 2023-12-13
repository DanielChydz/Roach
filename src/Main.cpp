#include "Main.hpp"

extern "C" void app_main(void){
  ESP_LOGI("Setup", "Rozpoczynanie konfiguracji.");
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // enable printf without \n
  setvbuf(stdout, NULL, _IONBF, 0);

  // setup wifi
  startWifiService();

  // setup motors and interrupts
  startMotorService();

  while(!connected) xTaskDelayUntil(&xLastWakeTime, 20);
  ESP_LOGI("Setup","Konfiguracja zakonczona. Rozpoczynanie dzialania programu.");

  while(true){
    xTaskDelayUntil(&xLastWakeTime, 20);
  }
}