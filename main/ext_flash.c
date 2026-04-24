#include "ext_flash.h"
#include "driver/spi_master.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "esp_check.h"
#include <dirent.h>

static const char *TAG = "ext_flash";

#define EXT_SPI_HOST  SPI2_HOST
#define EXT_CLK_IO    4
#define EXT_DO_IO     5   // MISO
#define EXT_DI_IO     6   // MOSI
#define EXT_CS_IO     7

static esp_flash_t *s_ext_flash;

esp_err_t ext_flash_init(void) {
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = EXT_DI_IO,
        .miso_io_num   = EXT_DO_IO,
        .sclk_io_num   = EXT_CLK_IO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXT_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "SPI bus init failed");

    esp_flash_spi_device_config_t dev_cfg = {
        .host_id   = EXT_SPI_HOST,
        .cs_id     = 0,
        .cs_io_num = EXT_CS_IO,
        .io_mode   = SPI_FLASH_DIO,
        .freq_mhz  = 40,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_flash_device(&s_ext_flash, &dev_cfg),
                        TAG, "Add flash device failed");
    ESP_RETURN_ON_ERROR(esp_flash_init(s_ext_flash),
                        TAG, "Flash init failed");

    uint32_t id;
    esp_flash_read_id(s_ext_flash, &id);
    ESP_LOGI(TAG, "External flash ID=0x%06lx  size=%lu bytes", (unsigned long)id, (unsigned long)s_ext_flash->size);

    const esp_partition_t *part;
    ESP_RETURN_ON_ERROR(
        esp_partition_register_external(s_ext_flash, 0, s_ext_flash->size,
                                        "ext_spiffs", ESP_PARTITION_TYPE_DATA,
                                        ESP_PARTITION_SUBTYPE_DATA_SPIFFS, &part),
        TAG, "Partition register failed");

    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path            = "/spiffs",
        .partition_label      = "ext_spiffs",
        .max_files            = 10,
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed (%s)", esp_err_to_name(err));
        ESP_LOGE(TAG, "Flash the SPIFFS image to the external chip first:");
        ESP_LOGE(TAG, "  esptool.py --chip esp32c3 --port <PORT> --spi-connection 4,5,6,7 write_flash 0x0 build/spiffs_ext.bin");
        return err;
    }

    size_t total = 0, used = 0;
    esp_spiffs_info("ext_spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%d  used=%d", total, used);

    DIR *d = opendir("/spiffs");
    if (d) {
        ESP_LOGI(TAG, "SPIFFS root contents:");
        struct dirent *ent;
        while ((ent = readdir(d)) != NULL)
            ESP_LOGI(TAG, "  %s", ent->d_name);
        closedir(d);
    } else {
        ESP_LOGW(TAG, "opendir(/spiffs) failed");
    }
    return ESP_OK;
}
