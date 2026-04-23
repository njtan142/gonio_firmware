#include "ina219.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_NUM             I2C_NUM_0
#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNT    0x01
#define INA219_REG_BUS      0x02
#define INA219_REG_POWER    0x03
#define INA219_REG_CURRENT  0x04
#define INA219_REG_CALIB    0x05

// current_lsb = 1mA, shunt = 0.1 ohm → cal = trunc(0.04096 / (0.001 * 0.1)) = 409
// If your board has a 0.01 ohm shunt, use 4096 instead
#define INA219_CALIB_VALUE  409
#define CURRENT_LSB_MA      1.0f
#define POWER_LSB_MW        20.0f

static esp_err_t ina219_write_reg(uint8_t reg, uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, value & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t ina219_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *value = (buf[0] << 8) | buf[1];
    }
    return ret;
}

esp_err_t ina219_init(void) {
    // 32V range, ±320mV shunt, 12-bit, continuous
    esp_err_t ret = ina219_write_reg(INA219_REG_CONFIG, 0x399F);
    if (ret != ESP_OK) return ret;
    return ina219_write_reg(INA219_REG_CALIB, INA219_CALIB_VALUE);
}

float ina219_read_bus_voltage(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_BUS, &raw);
    return ((raw >> 3) * 4) / 1000.0f;  // LSB = 4mV → convert to V
}

float ina219_read_current(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_CURRENT, &raw);
    return -(int16_t)raw * CURRENT_LSB_MA;
}

float ina219_read_power(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_POWER, &raw);
    return raw * POWER_LSB_MW;
}
