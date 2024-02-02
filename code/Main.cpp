#include "Main.hpp"

extern "C" void app_main(void){
  TickType_t xLastWakeTime = xTaskGetTickCount();

  ESP_ERROR_CHECK(gpio_set_direction(redLED, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_direction(greenLED, GPIO_MODE_OUTPUT));
  ESP_ERROR_CHECK(gpio_set_direction(blueLED, GPIO_MODE_OUTPUT));

  ESP_LOGI("Setup", "Rozpoczynanie konfiguracji.");

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