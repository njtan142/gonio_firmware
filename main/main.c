#include <stdio.h>

#define ENABLE_DISPLAY                                                         \
  1 // set to 0 to power off the display and measure baseline draw

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ina219.h"
#include "ssd1306_util.h"
#include "ext_flash.h"
#include "web_server.h"
#include "webrtc.h"

static const char *TAG = "coulomb";

#define BATTERY_CAPACITY_MAH  450.0
#define BATTERY_CAPACITY_MAS  (BATTERY_CAPACITY_MAH * 3600.0)

static double total_amp_seconds = 0.0;
static float  soc_percent       = 100.0f;
static SemaphoreHandle_t counter_mutex;
static SemaphoreHandle_t i2c_mutex;

// LiPo OCV → SoC lookup with linear interpolation
static float lipo_soc_from_voltage(float v) {
    static const float v_table[] = {3.00f, 3.40f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
    static const float s_table[] = {0.0f,  5.0f, 10.0f, 20.0f, 36.0f, 52.0f, 65.0f, 79.0f, 90.0f, 100.0f};
    const int n = 10;
    if (v <= v_table[0]) return 0.0f;
    if (v >= v_table[n - 1]) return 100.0f;
    for (int i = 1; i < n; i++) {
        if (v <= v_table[i]) {
            float t = (v - v_table[i - 1]) / (v_table[i] - v_table[i - 1]);
            return s_table[i - 1] + t * (s_table[i] - s_table[i - 1]);
        }
    }
    return 100.0f;
}

// Runs every 10ms — integrates current over time (trapezoidal accumulation)
static void coulomb_task(void *pv) {
  int64_t last_ts = 0;

  while (1) {
    int64_t now = esp_timer_get_time(); // microseconds

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    float current = ina219_read_current(); // mA
    xSemaphoreGive(i2c_mutex);

    if (last_ts != 0) {
      double dt = (double)(now - last_ts) / 1000000.0; // seconds
      xSemaphoreTake(counter_mutex, portMAX_DELAY);
      total_amp_seconds += current * dt;
      xSemaphoreGive(counter_mutex);
    }
    last_ts = now;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Runs every 1s — computes mAh consumed in the last second and updates display
static void ui_task(void *pv) {
  double prev_total = 0.0;
  char line[32];

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    xSemaphoreTake(counter_mutex, portMAX_DELAY);
    double snapshot = total_amp_seconds;
    xSemaphoreGive(counter_mutex);

    double delta_mas    = snapshot - prev_total;
    double mah_per_sec  = delta_mas / 3600.0;
    prev_total = snapshot;

    soc_percent -= (float)(delta_mas / BATTERY_CAPACITY_MAS * 100.0);
    if (soc_percent < 0.0f)   soc_percent = 0.0f;
    if (soc_percent > 100.0f) soc_percent = 100.0f;
    webrtc_set_soc(soc_percent);

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    float voltage    = ina219_read_bus_voltage();
    float current_ma = ina219_read_current();
    xSemaphoreGive(i2c_mutex);

    ESP_LOGI(TAG, "%.2fV  SoC:%.0f%%  %.1fmA  %.4f mAh/s",
             voltage, soc_percent, current_ma, mah_per_sec);

#if ENABLE_DISPLAY
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    ssd1306_clear_buffer();

    snprintf(line, sizeof(line), "%.2fV   SoC:%.0f%%", voltage, soc_percent);
    ssd1306_set_text(0, 0, line);

    snprintf(line, sizeof(line), "%.4f mAh/s", mah_per_sec);
    ssd1306_set_text(1, 0, line);

    ssd1306_set_text(2, 0, "ESP32-Monitor");
    ssd1306_set_text(3, 0, "192.168.4.1");

    ssd1306_update_display();
    xSemaphoreGive(i2c_mutex);
#endif
  }
}

void app_main(void) {
  i2c_master_init();
#if ENABLE_DISPLAY
  ssd1306_init();
#endif
  ina219_init();

  // Boot-time SoC estimate from OCV before any load settles
  soc_percent = lipo_soc_from_voltage(ina219_read_bus_voltage());

  if (ext_flash_init() == ESP_OK) {
    wifi_init_softap();
    web_server_start();
    webrtc_init();
  }

  counter_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  // coulomb_task at higher priority so sampling isn't starved by display work
  xTaskCreate(coulomb_task, "coulomb", 2048, NULL, 5, NULL);
  xTaskCreate(ui_task, "ui", 3072, NULL, 4, NULL);
}
