#include <string.h>
#include "ssd1306_util.h"
#include "font8x8_basic.h"

// Off-screen frame buffer where we draw before flushing to the physical display.
// Each bit represents one pixel (128x64 / 8 bits = 1024 bytes).
static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/**
 * @brief Writes a byte to the SSD1306 display via I2C.
 *
 * @param reg Register address (0x00 for command, 0x40 for data).
 * @param data Byte to write.
 * @return ESP_OK on success.
 */
static esp_err_t ssd1306_write_byte(uint8_t reg, uint8_t data) {
    // Create an I2C command link to queue up our multi-step transaction.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Write target address + write flag bit.
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    
    // Send the control byte (0x00 = command, 0x40 = data).
    i2c_master_write_byte(cmd, reg, true);
    // Send the actual byte to be written.
    i2c_master_write_byte(cmd, data, true);
    
    i2c_master_stop(cmd);
    // Begin the I2C transmission with a 1-second timeout.
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Sends a command byte to the SSD1306 display.
 *
 * @param command The command byte.
 */
static void ssd1306_command(uint8_t command) {
    ssd1306_write_byte(0x00, command);
}


/**
 * @brief Initializes the shared I2C master bus.
 */
void i2c_master_init(void) {
    // Configure the shared I2C bus that will be used by both the SSD1306 OLED and the INA219 power monitor.
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER, // Set the ESP32 to act as the I2C master
        .sda_io_num = I2C_MASTER_SDA_IO, // Data line GPIO pin mapping
        .scl_io_num = I2C_MASTER_SCL_IO, // Clock line GPIO pin mapping
        // Enable the ESP32's internal pull-up resistors on both the SDA and SCL lines.
        // This is typically required for the I2C bus to correctly idle in the HIGH state.
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set the clock frequency (e.g., 400kHz fast mode)
    };
    
    // Apply the hardware configuration parameters to the specified I2C controller port
    i2c_param_config(I2C_MASTER_NUM, &conf);
    
    // Install the driver into the OS. We disable the RX and TX software buffers
    // (set to 0) because we are operating in master mode, which manages transfers directly.
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief Initializes the SSD1306 OLED display.
 *
 * Sends the initialization sequence and clears the display buffer.
 */
void ssd1306_init(void) {
    // Initialization Sequence
    ssd1306_command(0xAE); // Display OFF
    ssd1306_command(0xD5); // Set Display Clock Divide Ratio
    ssd1306_command(0x80); // Suggested ratio
    ssd1306_command(0xA8); // Set Multiplex Ratio
    ssd1306_command(0x1F); // 1/32 Duty
    ssd1306_command(0xD3); // Set Display Offset
    ssd1306_command(0x00); // 0
    ssd1306_command(0x40); // Set Start Line 0
    ssd1306_command(0x8D); // Charge Pump
    ssd1306_command(0x14); // Enable Charge Pump
    ssd1306_command(0x20); // Memory Addressing Mode
    ssd1306_command(0x00); // Horizontal addressing mode
    ssd1306_command(0xA1); // Segment Remap 0 to 127
    ssd1306_command(0xC8); // COM Output Scan Direction
    ssd1306_command(0xDA); // Set COM Pins Hardware Config
    ssd1306_command(0x02); // 
    ssd1306_command(0x81); // Set Contrast
    ssd1306_command(0x8F);
    ssd1306_command(0xD9); // Set Pre-charge Period
    ssd1306_command(0xF1);
    ssd1306_command(0xDB); // Set VCOMH Deselect Level
    ssd1306_command(0x40);
    ssd1306_command(0xA4); // Entire Display Resume
    ssd1306_command(0xA6); // Normal Display
    ssd1306_command(0xAF); // Display ON
    
    // Clear Buffer
    memset(buffer, 0, sizeof(buffer));
    ssd1306_update_display();
}

/**
 * @brief Clears the internal display buffer.
 */
void ssd1306_clear_buffer(void) {
    memset(buffer, 0, sizeof(buffer));
}

/**
 * @brief Clears both the internal buffer and updates the physical display.
 */
void ssd1306_display_clear(void) {
    ssd1306_clear_buffer();
    ssd1306_update_display();
}

/**
 * @brief Draws a single pixel to the internal buffer.
 *
 * @param x X coordinate (0-127).
 * @param y Y coordinate (0-63 for 128x64 or 0-31 for 128x32).
 * @param color 1 for ON, 0 for OFF.
 */
void ssd1306_draw_pixel(int x, int y, int color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    
    // The SSD1306 RAM structure isn't a standard linear framebuffer. 
    // It's organized into "pages" that are 8 pixels tall. Each byte represents a vertical column of 8 pixels.
    // x determines the column offset, and (y / 8) determines which of the horizontal pages we are in.
    int index = x + (y / 8) * SSD1306_WIDTH;
    
    // The specific pixel inside that byte is addressed by the remainder of the Y coordinate (y % 8).
    if (color) {
        buffer[index] |= (1 << (y % 8));  // Set the bit to turn the pixel ON
    } else {
        buffer[index] &= ~(1 << (y % 8)); // Clear the bit to turn the pixel OFF
    }
}

/**
 * @brief Draws a single 8x8 character to the internal buffer.
 *
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param c The character to draw.
 */
void ssd1306_draw_char(int x, int y, char c) {
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8) return;
    
    // Check range
    // if (c < 0 || c > 127) c = '?'; // Removing this check as char might be unsigned or signed depending on compiler

    for (int i = 0; i < 8; i++) {
        // The 8x8 font array provides 8 bytes per character.
        // Each byte corresponds to a column (i), and the bits in the byte represent the rows (j).
        uint8_t line = font8x8_basic[(uint8_t)c][i];
        for (int j = 0; j < 8; j++) {
            // Check if the j-th bit is set, indicating a filled pixel for this font glyph.
            if (line & (1 << j)) {
                ssd1306_draw_pixel(x + i, y + j, 1);
            }
        }
    }
}

/**
 * @brief Writes a string of text to the internal buffer at a specific page and column.
 *
 * @param page The text page (Y position divided by 8).
 * @param col The X column coordinate.
 * @param text The null-terminated string to draw.
 */
void ssd1306_set_text(int page, int col, char *text) {
    int x = col;
    int y = page * 8;
    // Iterate through each character of the string until we hit the null terminator.
    while (*text) {
        // Render the current ASCII character at the current (x, y) buffer coordinates.
        ssd1306_draw_char(x, y, *text);
        
        // Advance the X coordinate by 8 pixels (the fixed width of the 8x8 font).
        x += 8;
        
        // Check for right-side boundary overflow.
        if (x >= SSD1306_WIDTH) {
            // Wrap the text to the next line: reset X to the left edge and move Y down by 8 pixels.
            x = 0;
            y += 8;
        }
        
        // Move to the next character in the source string.
        text++;
    }
}

/**
 * @brief Writes text to the buffer and immediately updates the display.
 *
 * @param page The text page (Y position divided by 8).
 * @param col The X column coordinate.
 * @param text The null-terminated string to draw.
 */
void ssd1306_display_text(int page, int col, char *text) {
    ssd1306_set_text(page, col, text);
    ssd1306_update_display();
}

/**
 * @brief Flushes the internal buffer to the physical OLED display.
 */
void ssd1306_update_display(void) {
    // The SSD1306 memory is organized into 8 "pages," each representing a horizontal strip 8 pixels high.
    for (uint8_t i = 0; i < SSD1306_HEIGHT / 8; i++) {
        // Set the target page address (0xB0 to 0xB7) where the next data will be written.
        ssd1306_command(0xB0 + i); 
        // Reset the column address to the start of the line (Column 0).
        ssd1306_command(0x00);     // Set Lower Column Start Address
        ssd1306_command(0x10);     // Set Higher Column Start Address

        // Batch the pixel data for the entire 128-pixel row into a single I2C transaction.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        
        // Use control byte 0x40 to indicate that following bytes should be written to Display RAM.
        i2c_master_write_byte(cmd, 0x40, true); 
        
        // Stream 128 bytes from our internal buffer. Each byte represents a vertical column of 8 pixels.
        i2c_master_write(cmd, &buffer[SSD1306_WIDTH * i], SSD1306_WIDTH, true);
        
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);
    }
}
