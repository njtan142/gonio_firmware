#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "ssd1306_util.h"
#include "ina219.h"

static const char *TAG = "coulomb";

static double           total_amp_seconds = 0.0;
static SemaphoreHandle_t counter_mutex;
static SemaphoreHandle_t i2c_mutex;

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

        // mA*s in the last 1s window → convert to mAh
        double mah_per_second = (snapshot - prev_total) / 3600.0;
        prev_total = snapshot;

        ESP_LOGI(TAG, "%.4f mAh/s", mah_per_second);

        snprintf(line, sizeof(line), "%.4f mAh/s", mah_per_second);

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        ssd1306_display_clear();
        ssd1306_display_text(0, 0, line);
        xSemaphoreGive(i2c_mutex);
    }
}

void app_main(void) {
    ssd1306_init();
    ina219_init();

    counter_mutex = xSemaphoreCreateMutex();
    i2c_mutex     = xSemaphoreCreateMutex();

    // coulomb_task at higher priority so sampling isn't starved by display work
    xTaskCreate(coulomb_task, "coulomb", 2048, NULL, 5, NULL);
    xTaskCreate(ui_task,      "ui",      3072, NULL, 4, NULL);
}
