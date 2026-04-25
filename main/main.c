#include <stdio.h>

#include "app_config.h"
#include "app_runtime.h"
#if !ENABLE_SYSTEM_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "ext_flash.h"
#include "mt6701.h"
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

/**
 * @brief System entry point and hardware bootstrap sequence.
 *
 * This coordinates the entire bring-up sequence. It strictly adheres to a dependency order:
 * 1. `i2c_master_init`: Must happen first because both the OLED and INA219 share this bus.
 * 2. `ina219_init` -> `lipo_soc_from_voltage`: Reads the battery Open Circuit Voltage (OCV) 
 *    before the heavy network loads drag down the voltage, ensuring an accurate initial SoC estimate.
 * 3. `ext_flash_init`: Attempts to mount the web assets.
 * 4. Network Stack: We only spin up the SoftAP, HTTP server, and WebRTC stack *if* the SPIFFS 
 *    mount succeeds, because the browser cannot load the WebRTC client without the HTML/JS.
 * 5. FreeRTOS Tasks: Finally spawns the 100Hz `coulomb_task` and the 1Hz `ui_task` with 
 *    appropriately staggered priorities so I2C polling doesn't starve the OLED rendering.
 */
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
    // SPI2 bus is live after ext_flash_init(); add MT6701 devices before
    // streaming_task starts reading them inside webrtc_init().
    mt6701_init();
    webrtc_init();
  }

  // Separate mutexes: one for sensor accumulator state, one for I2C bus access.
  counter_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  // coulomb_task at higher priority so sampling isn't starved by display work
  xTaskCreate(coulomb_task, "coulomb", 2048, NULL, 5, NULL);
  xTaskCreate(ui_task, "ui", 3072, NULL, 4, NULL);
}
