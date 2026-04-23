#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306_util.h"

static const char *TAG = "example";

void app_main(void)
{
    // Initialize I2C and SSD1306
    ssd1306_init();
    ssd1306_display_clear();
    ssd1306_display_text(0, 0, "Hello World from");
    ssd1306_display_text(1, 0, "ESP32-C3!");

    while (1) {
        ESP_LOGI(TAG, "Hello World");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}