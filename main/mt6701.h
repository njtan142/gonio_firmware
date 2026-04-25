#pragma once
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#define MT6701_NUM_SENSORS 4

// Chip-select GPIOs — CLK (GPIO 4) and MISO/DO (GPIO 5) are shared with the
// W25Q64 external flash on SPI2.
//
// Sensor → joint mapping (default gait configuration):
//   0  GPIO  0  Knee  — Central Node, always wired
//   1  GPIO  1  Hip yaw
//   2  GPIO  2  Hip pitch
//   3  GPIO 10  Ankle yaw
#define MT6701_CS0_IO  0
#define MT6701_CS1_IO  1
#define MT6701_CS2_IO  2
#define MT6701_CS3_IO 10

// Must be called after ext_flash_init() (which initialises the SPI2 bus).
// Probes each sensor; sensor 0 is required, sensors 1–3 are optional.
esp_err_t mt6701_init(void);

// Returns angle in degrees [0, 360). Returns 0.0 immediately for absent
// sensors. On SPI/CRC failure on a present sensor, returns the last valid
// reading and sets the error flag.
float mt6701_get_degrees(int sensor);

// True if the sensor was detected at init time.
bool mt6701_is_present(int sensor);

// True when the most recent read had a CRC fault or weak/strong-field warning,
// or the sensor was never detected.
bool mt6701_has_error(int sensor);
