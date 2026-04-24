#include <stdio.h>

#include "app_config.h"
#include "app_runtime.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ext_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ina219.h"
#include "ssd1306_util.h"
#include "web_server.h"
#include "webrtc.h"

const char *TAG = "coulomb";

// Accumulator in mA*s (milliamp-seconds). This is integrated in coulomb_task
// and read once per second by ui_task.
double total_amp_seconds = 0.0;
float soc_percent = 100.0f;
SemaphoreHandle_t counter_mutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL;

void app_main(void) {
  // Bring up shared I2C before any peripheral init that depends on it.
  i2c_master_init();
#if ENABLE_DISPLAY
  ssd1306_init();
#endif
  ina219_init();

  // Boot-time SoC estimate from OCV before any load settles
  soc_percent = lipo_soc_from_voltage(ina219_read_bus_voltage());

  if (ext_flash_init() == ESP_OK) {
    // Network stack and signaling are only useful if static assets mounted.
    wifi_init_softap();
    web_server_start();
    webrtc_init();
  }

  // Separate mutexes: one for sensor accumulator state, one for I2C bus access.
  counter_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  // coulomb_task at higher priority so sampling isn't starved by display work
  xTaskCreate(coulomb_task, "coulomb", 2048, NULL, 5, NULL);
  xTaskCreate(ui_task, "ui", 3072, NULL, 4, NULL);
}
