#ifndef SSD1306_UTIL_H
#define SSD1306_UTIL_H

#include <stdint.h>
#include "driver/i2c.h"

// I2C Hardware Configuration Parameters
// These define the shared I2C bus used by both the SSD1306 OLED display and the INA219 power monitor.
#define I2C_MASTER_SDA_IO           8      /*!< The physical GPIO pin number assigned to the I2C Serial Data (SDA) line. */
#define I2C_MASTER_SCL_IO           9      /*!< The physical GPIO pin number assigned to the I2C Serial Clock (SCL) line. */
#define I2C_MASTER_NUM              0      /*!< The hardware I2C controller port number to use (ESP32 typically has I2C_NUM_0 and I2C_NUM_1). */
#define I2C_MASTER_FREQ_HZ          400000 /*!< The I2C clock frequency in Hertz. 400kHz corresponds to I2C "Fast Mode". */
#define I2C_MASTER_TX_BUF_DISABLE   0      /*!< Set to 0 to disable the transmit ring buffer, as the I2C master manages transfers directly. */
#define I2C_MASTER_RX_BUF_DISABLE   0      /*!< Set to 0 to disable the receive ring buffer, as the I2C master manages transfers directly. */
#define I2C_MASTER_TIMEOUT_MS       1000   /*!< Maximum time in milliseconds to wait for an I2C transaction to complete before timing out. */

#define SSD1306_I2C_ADDRESS         0x3C
#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              32

void i2c_master_init(void);
void ssd1306_init(void);
void ssd1306_display_clear(void);
void ssd1306_clear_buffer(void);
void ssd1306_draw_pixel(int x, int y, int color);
void ssd1306_display_text(int page, int col, char *text);
void ssd1306_set_text(int page, int col, char *text);
void ssd1306_update_display(void);

#endif
