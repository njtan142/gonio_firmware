#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ssd1306_util.h"
#include "ina219.h"

static const char *TAG = "example";

void app_main(void)
{
    ssd1306_init();
    ina219_init();

    char line[32];

    while (1) {
        float voltage = ina219_read_bus_voltage();
        float current = ina219_read_current();
        float power   = ina219_read_power();

        ESP_LOGI(TAG, "Voltage: %.2fV  Current: %.1fmA  Power: %.1fmW", voltage, current, power);

        // TODO: causes blink — write to buffer directly and call ssd1306_update_display() once instead of clearing
        ssd1306_display_clear();
        snprintf(line, sizeof(line), "V: %.2f V", voltage);
        ssd1306_display_text(0, 0, line);
        snprintf(line, sizeof(line), "I: %.1f mA", current);
        ssd1306_display_text(1, 0, line);
        snprintf(line, sizeof(line), "P: %.1f mW", power);
        ssd1306_display_text(2, 0, line);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
