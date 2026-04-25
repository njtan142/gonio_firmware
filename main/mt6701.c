#include "mt6701.h"
#include "driver/spi_master.h"
#include "esp_log.h"

// MT6701 sensors share the SPI2 bus with the W25Q64 external flash.
// ext_flash.c initialises the bus (GPIO 4=CLK, 5=MISO, 6=MOSI); each sensor
// gets its own CS. The SPI master driver arbitrates bus access per device.
#define MT6701_SPI_HOST    SPI2_HOST
#define MT6701_FREQ_HZ     (1 * 1000 * 1000)  // 1 MHz — conservative for SSI on jumper wires
#define MT6701_PROBE_TRIES 5                   // reads used at boot to decide if a sensor is wired

// SSI frame bit layout (24 bits, MSB first):
//   [23:10] 14-bit angle     (0–16383 → 0–360°)
//   [ 9: 6]  4-bit mag status
//   [ 5: 0]  6-bit CRC
#define ANGLE_SHIFT  10
#define ANGLE_MASK   0x3FFF
#define STATUS_SHIFT 6
#define STATUS_MASK  0x0F
#define CRC_MASK     0x3F

// Mag-status bits that indicate a sensor fault.
#define MAG_WEAK_BIT   (1 << 2)
#define MAG_STRONG_BIT (1 << 3)

static const char *TAG = "mt6701";

// Human-readable joint names matching the default gait mapping.
static const char *k_joint_name[MT6701_NUM_SENSORS] = {
    "Knee (central)", "Hip-yaw", "Hip-pitch", "Ankle-yaw",
};

static const int k_cs_pins[MT6701_NUM_SENSORS] = {
    MT6701_CS0_IO, MT6701_CS1_IO, MT6701_CS2_IO, MT6701_CS3_IO,
};

static spi_device_handle_t s_devs[MT6701_NUM_SENSORS];
static float               s_last_deg[MT6701_NUM_SENSORS];
static bool                s_present[MT6701_NUM_SENSORS];
static bool                s_error[MT6701_NUM_SENSORS];

/**
 * @brief Computes a 6-bit CRC over 18 bits of sensor data.
 *
 * Uses polynomial G(x) = x^6 + x + 1 as specified in Datasheet §6.8.2.
 * The 6-bit feedback term (excluding the implicit leading x^6) is 0x03.
 *
 * @param data18 The 18 data bits to compute the CRC for (raw >> 6).
 * @return uint8_t The computed 6-bit CRC value.
 */
static uint8_t crc6_compute(uint32_t data18) {
    uint8_t crc = 0;
    // Process each bit of the 18-bit data, starting from the Most Significant Bit (MSB).
    for (int i = 17; i >= 0; i--) {
        // Extract the current input bit.
        uint8_t in = (data18 >> i) & 1;
        // Extract the feedback bit, which is the MSB of the current 6-bit CRC (bit 5).
        uint8_t fb = (crc >> 5) & 1;
        // Shift the CRC left by 1, dropping the old MSB and making room for the next bit.
        // We mask with 0x3F to ensure the CRC remains strictly 6 bits.
        crc = (crc << 1) & 0x3F;
        // If the input bit and feedback bit differ, we need to apply the polynomial.
        if (in ^ fb)
            // XOR with 0x03 (binary 000011). This represents the x^1 and x^0 terms 
            // of the polynomial G(x) = x^6 + x + 1. The x^6 term is implicit.
            crc ^= 0x03;
    }
    return crc;
}

/**
 * @brief Probes a specific sensor to check for electrical presence.
 *
 * Attempts up to MT6701_PROBE_TRIES reads and looks for a valid CRC.
 * A passing CRC proves the sensor is electrically responding; magnetic status
 * faults (e.g., weak/strong field) are handled at runtime instead.
 *
 * @param i Sensor index to probe (0 to MT6701_NUM_SENSORS - 1).
 * @return true if the sensor responds with a valid CRC, false otherwise.
 */
static bool probe_sensor(int i) {
    ESP_LOGI(TAG, "probing sensor %d (CS GPIO %d) ...", i, k_cs_pins[i]);
    for (int t = 0; t < MT6701_PROBE_TRIES; t++) {
        uint8_t rx[3] = {0};
        spi_transaction_t txn = {
            .length    = 24,
            .rxlength  = 24,
            .rx_buffer = rx,
        };
        if (spi_device_transmit(s_devs[i], &txn) != ESP_OK) {
            ESP_LOGW(TAG, "  [%d/%d] SPI transmit failed", t + 1, MT6701_PROBE_TRIES);
            continue;
        }
        // Assemble the 3 received bytes into a single 24-bit integer. (rx[0] is MSB, rx[2] is LSB)
        uint32_t raw      = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
        
        // Extract the 14-bit angle data (bits 23:10) by shifting right and applying the mask.
        uint16_t angle    = (raw >> ANGLE_SHIFT)  & ANGLE_MASK;
        
        // Extract the 4-bit magnetic status (bits 9:6) by shifting right and applying the mask.
        uint8_t  status   = (raw >> STATUS_SHIFT) & STATUS_MASK;
        
        // Extract the 6-bit received CRC (bits 5:0). No shift needed, just mask the lowest bits.
        uint8_t  crc_recv = raw & CRC_MASK;
        
        // Calculate the expected CRC using the upper 18 bits of the payload (angle + status).
        uint8_t  crc_calc = crc6_compute(raw >> STATUS_SHIFT);
        // 0xFFFFFF / 0x000000 are floating-bus signatures: MISO idles high or
        // low when no sensor is driving it, and all-ones/all-zeros happen to
        // produce a passing CRC. Reject both regardless of CRC result.
        bool     ok       = (crc_calc == crc_recv)
                          && (raw != 0xFFFFFF)
                          && (raw != 0x000000);
        ESP_LOGI(TAG, "  [%d/%d] raw=0x%06lx  angle=%u  status=0x%x  crc_recv=0x%02x  crc_calc=0x%02x  %s",
                 t + 1,
                 MT6701_PROBE_TRIES,
                 (unsigned long)raw,
                 angle,
                 status,
                 crc_recv,
                 crc_calc,
                 ok ? "PASS" : "FAIL");
        if (ok) return true;
    }
    ESP_LOGW(TAG, "  sensor %d: all %d probes failed", i, MT6701_PROBE_TRIES);
    return false;
}

/**
 * @brief Initializes the SPI bus and probes all MT6701 magnetic encoders.
 *
 * Configures the SPI interface for Mode 3 as required by the sensor to ensure
 * stable sampling. Iterates through all possible sensor slots, probing each
 * to establish presence and logging any missing sensors.
 *
 * @return ESP_OK on successful initialization.
 */
esp_err_t mt6701_init(void) {
    for (int i = 0; i < MT6701_NUM_SENSORS; i++) {
        spi_device_interface_config_t dev_cfg = {
            // Datasheet §6.8.1: CLK idles HIGH (CPOL=1). The sensor drives each
            // bit on the rising edge and holds it until the next rising edge.
            // A master sampling on the rising edge (CPHA=1) reads the stable
            // value set up on the previous cycle → SPI Mode 3 (CPOL=1, CPHA=1).
            // Mode 2 (sample on falling) hits the sensor mid-transition and
            // produces corrupt frames.
            .mode           = 3,
            .clock_speed_hz = MT6701_FREQ_HZ,
            .spics_io_num   = k_cs_pins[i],
            .queue_size     = 1,
        };
        esp_err_t err = spi_bus_add_device(MT6701_SPI_HOST, &dev_cfg, &s_devs[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "sensor %d: spi_bus_add_device failed: %s", i, esp_err_to_name(err));
            return err;
        }

        // Reset runtime state and probe if the sensor is electrically responding.
        s_last_deg[i] = 0.0f;
        s_error[i]    = false;
        s_present[i]  = probe_sensor(i);

        if (!s_present[i]) {
            s_error[i] = true;
            if (i == 0)
                // Sensor 0 is the central knee node; the system can still boot
                // but streaming will output zeros for this slot.
                ESP_LOGE(TAG, "sensor 0 (%s) not detected — check wiring", k_joint_name[0]);
            else
                ESP_LOGW(TAG, "sensor %d (%s) not detected — slot will output 0°",
                         i, k_joint_name[i]);
        } else {
            ESP_LOGI(TAG, "sensor %d (%s) present on CS GPIO %d",
                     i, k_joint_name[i], k_cs_pins[i]);
        }
    }
    return ESP_OK;
}

/**
 * @brief Reads the current absolute angle from the specified sensor.
 *
 * Performs a 24-bit SPI read and decodes the angle, status, and CRC fields.
 * If the sensor is absent, or if the CRC or magnetic status checks fail,
 * it sets an error flag and returns the last known good angle.
 *
 * @param sensor The sensor index to read.
 * @return float The current angle in degrees (0.0 to 360.0).
 */
float mt6701_get_degrees(int sensor) {
    if (sensor < 0 || sensor >= MT6701_NUM_SENSORS) return 0.0f;

    // Skip the SPI round-trip entirely for absent sensors.
    if (!s_present[sensor]) return 0.0f;

    uint8_t rx[3] = {0};
    spi_transaction_t t = {
        .length    = 24,   // clock out 24 bits; MOSI don't-care for read-only SSI
        .rxlength  = 24,
        .rx_buffer = rx,
    };

    if (spi_device_transmit(s_devs[sensor], &t) != ESP_OK) {
        // Set error flag and return the last known good position due to transmission failure.
        s_error[sensor] = true;
        return s_last_deg[sensor];
    }

    // Assemble the 3 received bytes into a single 24-bit integer. (rx[0] is MSB, rx[2] is LSB)
    uint32_t raw    = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    
    // Extract the 14-bit angle data (bits 23:10) by shifting right and applying the mask.
    uint16_t angle  = (raw >> ANGLE_SHIFT) & ANGLE_MASK;
    
    // Extract the 4-bit magnetic status (bits 9:6) by shifting right and applying the mask.
    uint8_t  status = (raw >> STATUS_SHIFT) & STATUS_MASK;
    
    // Extract the 6-bit received CRC (bits 5:0). No shift needed, just mask the lowest bits.
    uint8_t  crc    = raw & CRC_MASK;

    if (crc6_compute(raw >> STATUS_SHIFT) != crc) {
        ESP_LOGD(TAG, "sensor %d CRC fail raw=0x%06lx", sensor, (unsigned long)raw);
        // Set error flag and return the last known good position due to data corruption.
        s_error[sensor] = true;
        return s_last_deg[sensor];
    }

    s_error[sensor]    = !!(status & (MAG_WEAK_BIT | MAG_STRONG_BIT));
    s_last_deg[sensor] = angle * (360.0f / 16384.0f);
    return s_last_deg[sensor];
}

/**
 * @brief Checks if a specific sensor was detected during initialization.
 *
 * @param sensor The sensor index to check.
 * @return true if the sensor is present, false otherwise.
 */
bool mt6701_is_present(int sensor) {
    if (sensor < 0 || sensor >= MT6701_NUM_SENSORS) return false;
    return s_present[sensor];
}

/**
 * @brief Checks if the specified sensor has encountered a recent read error.
 *
 * This includes SPI transmission failures, CRC mismatches, or magnetic field
 * strength warnings reported by the sensor hardware.
 *
 * @param sensor The sensor index to check.
 * @return true if an error has occurred, false otherwise.
 */
bool mt6701_has_error(int sensor) {
    if (sensor < 0 || sensor >= MT6701_NUM_SENSORS) return true;
    return s_error[sensor];
}
