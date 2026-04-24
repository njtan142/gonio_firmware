#include <stdio.h>

#include "app_config.h"
#include "app_runtime.h"
#if !ENABLE_SYSTEM_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ina219.h"
#include "ssd1306_util.h"
#include "webrtc.h"

extern const char *TAG;
extern double total_amp_seconds;
extern float soc_percent;
extern SemaphoreHandle_t counter_mutex;
extern SemaphoreHandle_t i2c_mutex;

/**
 * @brief Converts LiPo open-circuit voltage to State of Charge (SoC).
 *
 * Uses a lookup table with linear interpolation.
 *
 * @param v Voltage reading.
 * @return float State of Charge percentage (0.0 to 100.0).
 */
float lipo_soc_from_voltage(float v) {
  static const float v_table[] = {3.00f, 3.40f, 3.50f, 3.60f, 3.70f,
                                  3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float s_table[] = {0.0f,  5.0f,  10.0f, 20.0f, 36.0f,
                                  52.0f, 65.0f, 79.0f, 90.0f, 100.0f};
  const int n = 10;
  if (v <= v_table[0])
    return 0.0f;
  if (v >= v_table[n - 1])
    return 100.0f;
  // Iterate through the voltage breakpoints to find the correct interpolation segment.
  for (int i = 1; i < n; i++) {
    if (v <= v_table[i]) {
      // Calculate 't', the normalized position (0.0 to 1.0) of the voltage between the lower and upper bounds.
      float t = (v - v_table[i - 1]) / (v_table[i] - v_table[i - 1]);
      // Interpolate the SoC using 't' to smoothly transition between the table's fixed percentage points.
      return s_table[i - 1] + t * (s_table[i] - s_table[i - 1]);
    }
  }
  return 100.0f;
}

/**
 * @brief FreeRTOS task for Coulomb counting.
 *
 * Runs every 10ms. Integrates current over time (trapezoidal accumulation).
 *
 * @param pv Task parameter (unused).
 */
void coulomb_task(void *pv) {
  int64_t last_ts = 0;

  while (1) {
    int64_t now = esp_timer_get_time(); // microseconds

    // Acquire the shared I2C bus lock to safely query the INA219.
    // This prevents collisions if the UI task tries to update the OLED at the same time.
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    float current = ina219_read_current(); // Measured in milliamperes (mA)
    xSemaphoreGive(i2c_mutex);

    // Skip integration on the very first sample because we don't have a valid previous timestamp (dt is unknown).
    if (last_ts != 0) {
      // Calculate delta time (dt) in seconds. esp_timer_get_time() returns microseconds.
      double dt = (double)(now - last_ts) / 1000000.0;
      
      // Accumulate the consumed charge into the global tracker.
      // This is a basic Riemann sum integration (Current * Time = Charge in mA*s).
      // We lock counter_mutex so the UI task doesn't read a partially updated value during this math.
      xSemaphoreTake(counter_mutex, portMAX_DELAY);
      total_amp_seconds += current * dt;
      xSemaphoreGive(counter_mutex);
    }
    last_ts = now;

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
 * @brief FreeRTOS task for UI and telemetry updates.
 *
 * Runs every 1s. Computes mAh consumed in the last second and updates the OLED display
 * and WebRTC telemetry.
 *
 * @param pv Task parameter (unused).
 */
void ui_task(void *pv) {
  double prev_total = 0.0;
  char line[32];

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Snapshot the accumulator under lock so the UI and HTTP systems never
    // read a tearing/corrupted value while the 100Hz sampler task is writing to it.
    xSemaphoreTake(counter_mutex, portMAX_DELAY);
    double snapshot = total_amp_seconds;
    xSemaphoreGive(counter_mutex);

    // Calculate how much charge (in mA*s) was consumed since the last 1-second UI tick.
    double delta_mas = snapshot - prev_total;
    
    // Convert mA*s (milliamp-seconds) to mAh (milliamp-hours) for standard, human-readable telemetry.
    // 3600 seconds = 1 hour.
    double mah_per_sec = delta_mas / 3600.0;
    prev_total = snapshot;

    // Active Coulomb counting: subtract the strictly consumed charge from the remaining battery capacity.
    // BATTERY_CAPACITY_MAS represents the total expected battery capacity scaled to milliamp-seconds.
    soc_percent -= (float)(delta_mas / BATTERY_CAPACITY_MAS * 100.0);
    if (soc_percent < 0.0f)
      soc_percent = 0.0f;
    if (soc_percent > 100.0f)
      soc_percent = 100.0f;
    webrtc_set_soc(soc_percent);

    // INA219 and SSD1306 share the same I2C controller.
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    float voltage = ina219_read_bus_voltage();
    float current_ma = ina219_read_current();
    xSemaphoreGive(i2c_mutex);

 #if ENABLE_COULOMB_LOG
    ESP_LOGI(TAG, "%.2fV  SoC:%.0f%%  %.1fmA  %.4f mAh/s", voltage, soc_percent,
             current_ma, mah_per_sec);
 #endif

#if ENABLE_DISPLAY
    // Keep the display update as one critical section to avoid interleaved
    // transactions with other I2C users.
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
