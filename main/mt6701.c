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

// CRC-6 over the 18 data bits (raw >> 6), polynomial G(x) = x^6 + x + 1.
// Datasheet §6.8.2: "CRC polynomials: X6+X+1, MSB stream in first."
// The 6-bit feedback term (without the implicit leading x^6) is 0x03.
static uint8_t crc6_compute(uint32_t data18) {
    uint8_t crc = 0;
    for (int i = 17; i >= 0; i--) {
        uint8_t in = (data18 >> i) & 1;
        uint8_t fb = (crc >> 5) & 1;
        crc = (crc << 1) & 0x3F;
        if (in ^ fb)
            crc ^= 0x03;
    }
    return crc;
}

// Returns true if at least one of MT6701_PROBE_TRIES reads returns a valid CRC.
// A passing CRC proves the sensor is electrically responding; magnetic status
// faults (weak/strong field) are a separate concern handled at runtime.
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
        uint32_t raw      = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
        uint16_t angle    = (raw >> ANGLE_SHIFT)  & ANGLE_MASK;
        uint8_t  status   = (raw >> STATUS_SHIFT) & STATUS_MASK;
        uint8_t  crc_recv = raw & CRC_MASK;
        uint8_t  crc_calc = crc6_compute(raw >> STATUS_SHIFT);
        // 0xFFFFFF / 0x000000 are floating-bus signatures: MISO idles high or
        // low when no sensor is driving it, and all-ones/all-zeros happen to
        // produce a passing CRC. Reject both regardless of CRC result.
        bool     ok       = (crc_calc == crc_recv)
                          && (raw != 0xFFFFFF)
                          && (raw != 0x000000);
        ESP_LOGI(TAG, "  [%d/%d] raw=0x%06lx  angle=%u  status=0x%x  crc_recv=0x%02x  crc_calc=0x%02x  %s",
                 t + 1, MT6701_PROBE_TRIES,
                 (unsigned long)raw, angle, status,
                 crc_recv, crc_calc,
                 ok ? "PASS" : "FAIL");
        if (ok) return true;
    }
    ESP_LOGW(TAG, "  sensor %d: all %d probes failed", i, MT6701_PROBE_TRIES);
    return false;
}

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
        s_error[sensor] = true;
        return s_last_deg[sensor];
    }

    uint32_t raw    = ((uint32_t)rx[0] << 16) | ((uint32_t)rx[1] << 8) | rx[2];
    uint16_t angle  = (raw >> ANGLE_SHIFT) & ANGLE_MASK;
    uint8_t  status = (raw >> STATUS_SHIFT) & STATUS_MASK;
    uint8_t  crc    = raw & CRC_MASK;

    if (crc6_compute(raw >> STATUS_SHIFT) != crc) {
        ESP_LOGD(TAG, "sensor %d CRC fail raw=0x%06lx", sensor, (unsigned long)raw);
        s_error[sensor] = true;
        return s_last_deg[sensor];
    }

    s_error[sensor]    = !!(status & (MAG_WEAK_BIT | MAG_STRONG_BIT));
    s_last_deg[sensor] = angle * (360.0f / 16384.0f);
    return s_last_deg[sensor];
}

bool mt6701_is_present(int sensor) {
    if (sensor < 0 || sensor >= MT6701_NUM_SENSORS) return false;
    return s_present[sensor];
}

bool mt6701_has_error(int sensor) {
    if (sensor < 0 || sensor >= MT6701_NUM_SENSORS) return true;
    return s_error[sensor];
}
