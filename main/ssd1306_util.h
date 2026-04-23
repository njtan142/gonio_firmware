#ifndef SSD1306_UTIL_H
#define SSD1306_UTIL_H

#include <stdint.h>
#include "driver/i2c.h"

// I2C Configuration
#define I2C_MASTER_SDA_IO           8      /*!< GPIO number used for I2C master data */
#define I2C_MASTER_SCL_IO           9      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_NUM              0      /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          400000 /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0      /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

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
