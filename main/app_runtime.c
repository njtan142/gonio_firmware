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
#include "hfhl_ws.h"
#include "ina219.h"
#include "mt6701.h"
#include "ssd1306_util.h"
#include "web_server_api.h"
#include "webrtc.h"
#include <math.h>

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
 * @brief FreeRTOS task for SoC accounting and WebRTC telemetry.
 *
 * Runs every 1s. Computes mAh consumed in the last second, updates the global
 * soc_percent, and notifies WebRTC. Does NOT touch the OLED — that is handled
 * by display_task at a higher rate.
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

  }
}

/**
 * @brief FreeRTOS task for high-rate OLED display updates.
 *
 * Runs at 20 Hz (every 50 ms). Reads sensor angles over SPI (no I2C load),
 * snapshots soc_percent under a brief mutex, then renders the full screen in
 * a single I2C burst. Completely decoupled from the 1 Hz SoC accounting.
 *
 * @param pv Task parameter (unused).
 */
static float apply_lut(float raw, const sensor_lut_t *lut) {
    if (lut->num_points == 0) return raw;
    if (lut->num_points == 1) return raw - lut->points[0].raw + lut->points[0].physical;

    int n = lut->num_points;
    for (int i = 0; i < n; i++) {
        int next = (i + 1) % n;
        float r0 = lut->points[i].raw;
        float r1 = lut->points[next].raw;
        
        float dr = r1 - r0;
        while (dr < 0.0f) dr += 360.0f;
        while (dr >= 360.0f) dr -= 360.0f;
        
        float d_raw = raw - r0;
        while (d_raw < 0.0f) d_raw += 360.0f;
        while (d_raw >= 360.0f) d_raw -= 360.0f;
        
        if (d_raw <= dr || n == 2) { 
            if (dr == 0.0f) return lut->points[i].physical;
            float t = d_raw / dr;
            float p0 = lut->points[i].physical;
            const float p1 = lut->points[next].physical;
            float dp = p1 - p0;
            // Choose the dp that is closest to the raw distance dr to ensure
            // a consistent rotation direction and prevent 'flipping' at the wrap.
            while (dp - dr < -180.0f) dp += 360.0f;
            while (dp - dr > 180.0f) dp -= 360.0f;
            float phys = p0 + t * dp;
            while (phys < 0.0f) phys += 360.0f;
            while (phys >= 360.0f) phys -= 360.0f;
            return phys;
        }
    }
    return raw;
}

void display_task(void *pv) {
#if ENABLE_DISPLAY
  char line[32];

  while (1) {
    // 20 Hz — fast enough to feel live, but well within the ~12 ms I2C flush budget.
    vTaskDelay(pdMS_TO_TICKS(50));

    // Determine device state from connection flags (atomic bool reads, no mutex needed).
    const char *state_str;
    if (hfhl_ws_is_connected()) {
        state_str = "HFHL";
    } else if (webrtc_is_connected()) {
        state_str = "LFLL";
    } else {
        state_str = "IDLE";
    }

    // Read all 4 sensor angles via SPI, apply LUT, then zero offsets and scale factors.
    float ang[MT6701_NUM_SENSORS];
    for (int s = 0; s < MT6701_NUM_SENSORS; s++) {
        float raw = mt6701_get_degrees(s);
        float calibrated = apply_lut(raw, &g_luts[s]);
        float zeroed = (calibrated - g_zero_offsets[s]) * g_scale_factors[s];
        // Wrap to [0, 360)
        zeroed = fmodf(zeroed, 360.0f);
        if (zeroed < 0.0f) zeroed += 360.0f;
        ang[s] = zeroed;
    }

    // Snapshot soc_percent under the counter mutex (brief, non-blocking).
    xSemaphoreTake(counter_mutex, portMAX_DELAY);
    float soc_snap = soc_percent;
    xSemaphoreGive(counter_mutex);

    // Acquire I2C bus and flush the full frame in one critical section.
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    ssd1306_clear_buffer();

    // Row 0: Knee (S0) and Hip-Yaw (S1) angles
    snprintf(line, sizeof(line), "K:%5.1f H:%5.1f", ang[0], ang[1]);
    ssd1306_set_text(0, 0, line);

    // Row 1: Hip-Pitch (S2) and Ankle-Pitch (S3) angles
    snprintf(line, sizeof(line), "P:%5.1f A:%5.1f", ang[2], ang[3]);
    ssd1306_set_text(1, 0, line);

    // Row 2: State of Charge
    snprintf(line, sizeof(line), "SoC: %5.1f%%", soc_snap);
    ssd1306_set_text(2, 0, line);

    // Row 3: Device state
    snprintf(line, sizeof(line), "State: %s", state_str);
    ssd1306_set_text(3, 0, line);

    ssd1306_update_display();
    xSemaphoreGive(i2c_mutex);
  }
#else
  vTaskDelete(NULL); // display disabled — self-terminate cleanly
#endif
}
