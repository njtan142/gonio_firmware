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

/**
 * @brief Writes a 16-bit value to an INA219 register.
 *
 * @param reg Register address.
 * @param value 16-bit value to write.
 * @return ESP_OK on success.
 */
static esp_err_t ina219_write_reg(uint8_t reg, uint16_t value) {
    // Allocate an I2C command link to queue up our transaction sequence.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Send the INA219 target address and set the WRITE flag.
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    // Specify the target internal register address to write to.
    i2c_master_write_byte(cmd, reg, true);
    
    // The INA219 expects 16-bit data in big-endian format (Most Significant Byte first).
    i2c_master_write_byte(cmd, (value >> 8) & 0xFF, true); // MSB
    i2c_master_write_byte(cmd, value & 0xFF, true);        // LSB
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Reads a 16-bit value from an INA219 register.
 *
 * @param reg Register address.
 * @param[out] value Pointer to store the read value.
 * @return ESP_OK on success.
 */
static esp_err_t ina219_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    // Set up an I2C "repeated start" transaction sequence to safely read without giving up bus arbitration.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Phase 1: Write the register pointer we want to read from.
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // Phase 2: Send a repeated start (without stop) to shift to read mode.
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (INA219_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
    
    // Read the 2-byte big-endian response. ACK the first byte to request the second.
    i2c_master_read_byte(cmd, &buf[0], I2C_MASTER_ACK);
    // NACK the final byte to tell the INA219 we are done reading.
    i2c_master_read_byte(cmd, &buf[1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *value = (buf[0] << 8) | buf[1];
    }
    return ret;
}

/**
 * @brief Initializes the INA219 current/power monitor sensor.
 *
 * Sets configuration and calibration registers.
 *
 * @return ESP_OK on success.
 */
esp_err_t ina219_init(void) {
    // 32V bus range, ±320mV shunt range, 12-bit ADC, continuous conversion.
    esp_err_t ret = ina219_write_reg(INA219_REG_CONFIG, 0x399F);
    if (ret != ESP_OK) return ret;
    // Calibration config defines current/power scaling used by read_current/power.
    return ina219_write_reg(INA219_REG_CALIB, INA219_CALIB_VALUE);
}

/**
 * @brief Reads the bus voltage from INA219.
 *
 * @return float Bus voltage in Volts.
 */
float ina219_read_bus_voltage(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_BUS, &raw);
    return ((raw >> 3) * 4) / 1000.0f;  // LSB = 4mV → convert to V
}

/**
 * @brief Reads the current flowing through the shunt resistor.
 *
 * @return float Current in milliamperes (mA).
 */
float ina219_read_current(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_CURRENT, &raw);
    // Board wiring makes discharge current negative; invert for positive draw.
    return -(int16_t)raw * CURRENT_LSB_MA;
}

/**
 * @brief Reads the power consumption from INA219.
 *
 * @return float Power in milliwatts (mW).
 */
float ina219_read_power(void) {
    uint16_t raw = 0;
    ina219_read_reg(INA219_REG_POWER, &raw);
    return raw * POWER_LSB_MW;
}
