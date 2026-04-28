#ifndef INA219_H
#define INA219_H

#include "esp_err.h"
#include <stdint.h>

#define INA219_I2C_ADDRESS                                                     \
  0x40 // The default 7-bit I2C slave address of the INA219 chip.

esp_err_t ina219_init(void);
float ina219_read_bus_voltage(void); // Volts
float ina219_read_current(void);     // mA
float ina219_read_power(void);       // mW

#endif
