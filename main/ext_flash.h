#pragma once
#include "esp_err.h"

// GPIO used as SPI MISO for the external flash (shared with MT6701 DO lines).
// Exposed here so callers can configure the pin (e.g. pull-up) before the bus
// is initialised.
#define EXT_DO_IO 5
#define EXT_CS_IO 7

esp_err_t ext_flash_init(void);
