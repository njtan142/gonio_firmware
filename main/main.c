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
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "ina219.h"
#include "ssd1306_util.h"
#include "web_server.h"
#include "webrtc.h"

const char *TAG = "system";

// Accumulator in mA*s (milliamp-seconds). This is integrated in coulomb_task
// and read once per second by ui_task.
double total_amp_seconds = 0.0;
float soc_percent = 100.0f;
SemaphoreHandle_t counter_mutex = NULL;
SemaphoreHandle_t i2c_mutex = NULL;

/**
 * @brief System entry point and hardware bootstrap sequence.
 *
 * Dependency order:
 * 1. `i2c_master_init`: Must happen first — INA219 shares this bus.
 * 2. `ina219_init` -> `lipo_soc_from_voltage`: OCV read before heavy loads settle.
 * 3. MT6701 CS pins HIGH + MISO pull-down: Prevents SPI bus interference during flash probe.
 * 4. `ext_flash_init`: Before wifi_init_softap() to avoid the WiFi current spike.
 * 5. `ssd1306_init`: Non-critical display init, deferred past the flash probe window.
 * 6. `wifi_init_softap`: Always starts for debuggability.
 * 7. HTTP server + MT6701 + WebRTC: Only if flash mounted.
 * 8. FreeRTOS tasks: coulomb_task (100 Hz) and ui_task (1 Hz).
 */
void app_main(void) {
  // Bring up I2C first — INA219 needs it for the OCV read below.
  // ssd1306_init() is intentionally deferred past the flash probe window
  // since the OLED is non-critical and its I2C traffic adds unnecessary
  // current draw during the sensitive flash initialisation window.
  i2c_master_init();
#if ENABLE_DISPLAY
  ssd1306_init();
  ssd1306_display_clear(); // blank the screen immediately on boot
#endif
  ina219_init();

  // Boot-time SoC estimate from OCV before any load settles.
  soc_percent = lipo_soc_from_voltage(ina219_read_bus_voltage());

  // PRE-CONDITION for ext_flash_init(): drive all MT6701 CS pins HIGH before
  // the SPI2 bus is initialised.  At reset, these GPIOs float and the MT6701
  // treats a low/floating CS as "selected", actively driving MISO.  With two
  // or more sensors simultaneously driving MISO during the flash probe, the
  // bus is corrupted -> "memspi: no response".  Asserting CS high here
  // deasserts every sensor before SPI2 even comes up.  mt6701_init() will
  // reconfigure the same pins as hardware-CS outputs once the bus is ready.
  static const int k_cs_gpios[] = {
      MT6701_CS0_IO, MT6701_CS1_IO, MT6701_CS2_IO, MT6701_CS3_IO,
      EXT_CS_IO // Also deassert the external flash CS!
  };
  static const char *BOOT_TAG = "boot";
  ESP_LOGI(BOOT_TAG, "deasserting all CS pins before flash probe");
  for (int i = 0; i < (int)(sizeof(k_cs_gpios) / sizeof(k_cs_gpios[0])); i++) {
    gpio_config_t cs_cfg = {
        .pin_bit_mask = 1ULL << k_cs_gpios[i],
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cs_cfg);
    gpio_set_level(k_cs_gpios[i], 1); // deassert — CS is active-low
    ESP_LOGI(BOOT_TAG, "  GPIO %d CS -> HIGH", k_cs_gpios[i]);
  }

  // Pull MISO low so we can detect if anything is driving it HIGH before probe.
  ESP_LOGI(BOOT_TAG, "enabling pull-down on MISO GPIO %d", EXT_DO_IO);
  gpio_config_t miso_cfg = {
      .pin_bit_mask = 1ULL << EXT_DO_IO,
      .mode         = GPIO_MODE_INPUT,
      .pull_up_en   = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type    = GPIO_INTR_DISABLE,
  };
  gpio_config(&miso_cfg);

  ESP_LOGI(BOOT_TAG, "waiting 20 ms for SPI lines to settle...");
  vTaskDelay(pdMS_TO_TICKS(20));

  // MISO level check: 0=free, 1=something driving HIGH against pull-down.
  int miso_level = gpio_get_level(EXT_DO_IO);
  ESP_LOGI(BOOT_TAG, "MISO (GPIO %d) = %d %s",
           EXT_DO_IO, miso_level,
           miso_level ? "(HIGH — driven)" : "(LOW — free)");



  // WiFi AP always starts so the chip is reachable even if flash failed.
  wifi_init_softap();

  // Wait for WiFi AP + power rail to fully settle before probing the flash.
  // With 2 MT6701 sensors the combined current draw stresses the LDO;
  // giving the supply 500 ms to stabilise after WiFi init improves flash
  // probe reliability.
  ESP_LOGI(BOOT_TAG, "waiting 2 s for power to settle after WiFi init...");
  vTaskDelay(pdMS_TO_TICKS(2000));
#if ENABLE_EXT_FLASH
  ESP_LOGI(BOOT_TAG, "probing external flash");
  bool flash_ok = (ext_flash_init() == ESP_OK);
  ESP_LOGI(BOOT_TAG, "ext_flash_init -> %s", flash_ok ? "OK" : "FAILED");

  if (flash_ok) {
    web_server_start();
    mt6701_init();
    webrtc_init();
  }
#else
  ESP_LOGI(BOOT_TAG, "Bypassing external flash to test MT6701...");
  
  // Initialize SPI2 bus directly since we bypassed ext_flash_init
  spi_bus_config_t bus_cfg = {
      .mosi_io_num = 6,  // MOSI
      .miso_io_num = 5,  // MISO
      .sclk_io_num = 4,  // CLK
      .max_transfer_sz = 4096,
      .flags = SPICOMMON_BUSFLAG_MASTER,
  };
  spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

  // We are not starting the web server since there is no flash memory
  // Start the sensors directly!
  mt6701_init();
  webrtc_init();
#endif

  // Separate mutexes: one for sensor accumulator state, one for I2C bus access.
  counter_mutex = xSemaphoreCreateMutex();
  i2c_mutex = xSemaphoreCreateMutex();

  // coulomb_task at higher priority so sampling isn't starved by display work
  xTaskCreate(coulomb_task, "coulomb", 2048, NULL, 5, NULL);
  xTaskCreate(ui_task, "ui", 3072, NULL, 4, NULL);
}

