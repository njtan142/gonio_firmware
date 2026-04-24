#include "ext_flash.h"
#include "driver/spi_master.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_spiffs.h"
#include "app_config.h"
#if !ENABLE_SYSTEM_LOG
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#endif
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

/**
 * @brief Initializes the external SPI NOR flash and mounts the static web assets.
 *
 * The ESP32-C3 has limited internal flash. To serve rich web content (HTML/JS/CSS), 
 * an external SPI NOR flash chip is wired to the SPI2 bus. This function initializes 
 * the SPI bus, probes the external flash chip, registers it as a data partition, 
 * and finally mounts the pre-built SPIFFS filesystem over it.
 *
 * @return ESP_OK on success, or an error code upon failure (e.g. if chip is missing or unformatted).
 */
esp_err_t ext_flash_init(void) {
    // 1. Initialize the SPI2 bus dedicated solely to the external NOR flash.
    // We do not share this bus with the I2C peripherals to avoid latency spikes.
    spi_bus_config_t bus_cfg = {
        .mosi_io_num   = EXT_DI_IO,   // Master Out Slave In
        .miso_io_num   = EXT_DO_IO,   // Master In Slave Out
        .sclk_io_num   = EXT_CLK_IO,  // SPI Clock
        // Quad SPI is disabled because the current wiring only uses standard 1-bit SPI (DI/DO).
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        // Set maximum transfer size to 4KB (the standard SPIFFS block size).
        // This allows reading an entire filesystem block in a single DMA transaction, improving throughput.
        .max_transfer_sz = 4096,
    };
    // Initialize the bus with automatic DMA channel selection. DMA is required for 4KB transfers.
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXT_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO),
                        TAG, "SPI bus init failed");

    // 2. Add the flash chip to the newly created SPI bus.
    esp_flash_spi_device_config_t dev_cfg = {
        .host_id   = EXT_SPI_HOST,
        .cs_id     = 0,               // Standard CS0
        .cs_io_num = EXT_CS_IO,       // Chip Select GPIO
        // We use SPI_FLASH_SLOWRD (Standard SPI read) at 10 MHz.
        // While faster modes exist, 10 MHz is highly stable on breadboards/jumper wires without signal integrity issues.
        .io_mode   = SPI_FLASH_SLOWRD,
        .freq_mhz  = 10,
    };
    ESP_RETURN_ON_ERROR(spi_bus_add_flash_device(&s_ext_flash, &dev_cfg),
                        TAG, "Add flash device failed");
                        
    // Probes the flash chip to verify it is awake and responding.
    ESP_RETURN_ON_ERROR(esp_flash_init(s_ext_flash),
                        TAG, "Flash init failed");

    uint32_t id;
    // Read the JEDEC ID. This provides physical proof that the MISO/MOSI/CLK lines are wired correctly.
    esp_flash_read_id(s_ext_flash, &id);
    ESP_LOGI(TAG, "External flash ID=0x%06lx  size=%lu bytes", (unsigned long)id, (unsigned long)s_ext_flash->size);

    const esp_partition_t *part;
    // 3. Expose the raw external chip memory as a logical ESP32 partition.
    // Unlike internal flash, external chips don't have a partition table flashed at 0x8000.
    // We programmatically map the entire chip (from offset 0 to size) as a single SPIFFS partition named "ext_spiffs".
    ESP_RETURN_ON_ERROR(
        esp_partition_register_external(s_ext_flash, 0, s_ext_flash->size,
                                        "ext_spiffs", ESP_PARTITION_TYPE_DATA,
                                        ESP_PARTITION_SUBTYPE_DATA_SPIFFS, &part),
        TAG, "Partition register failed");

    // Prepare the Virtual File System (VFS) configuration for SPIFFS.
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        // This is the VFS prefix. The HTTP server will map URLs directly onto this base path.
        .base_path            = "/spiffs",
        // This MUST exactly match the partition label we just registered above ("ext_spiffs").
        .partition_label      = "ext_spiffs",
        // Limit simultaneous open files to keep heap RAM fragmentation predictable and bounded.
        .max_files            = 10,
        // CRITICAL: Keep disabled. Our build process generates a prebuilt SPIFFS binary image
        // full of web assets. If we format it on failure, we lose all HTML/JS files permanently.
        .format_if_mount_failed = false,
    };
    
    esp_err_t err = esp_vfs_spiffs_register(&spiffs_cfg);
    if (err != ESP_OK) {
        // Fail fast. Auto-formatting would hide the fact that the SPIFFS image wasn't flashed properly.
        // It provides a direct hint to the developer via UART.
        ESP_LOGE(TAG, "SPIFFS mount failed (%s)", esp_err_to_name(err));
        ESP_LOGE(TAG, "Flash the SPIFFS image to the external chip first:");
        ESP_LOGE(TAG, "  esptool.py --chip esp32c3 --port <PORT> --spi-connection 4,5,6,7 write_flash 0x0 build/spiffs_ext.bin");
        return err;
    }

    size_t total = 0, used = 0;
    // Handy runtime sanity check after successful mount.
    esp_spiffs_info("ext_spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%d  used=%d", total, used);

    DIR *d = opendir("/spiffs");
    if (d) {
        // 5. Diagnostics: Dump the root directory contents. 
        // If this prints 'index.html' and other expected assets, the external flash is fully working
        // and the web server can safely start serving files.
        ESP_LOGI(TAG, "SPIFFS root contents:");
        struct dirent *ent;
        while ((ent = readdir(d)) != NULL)
            ESP_LOGI(TAG, "  %s", ent->d_name);
        closedir(d);
    } else {
        // If the mount succeeded but opendir fails, the filesystem might be corrupted.
        ESP_LOGW(TAG, "opendir(/spiffs) failed");
    }
    return ESP_OK;
}
