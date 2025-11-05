#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// OLED Display Configuration - Seeed XIAO ESP32S3 Expansion Board
// On the XIAO ESP32S3 + Expansion Board, the onboard OLED (SSD1306) is on I2C bus
// To access them, initialize SDA = D4 (GPIO5), SCL = D5 (GPIO6), address 0x3C

#define OLED_SDA_PIN GPIO_NUM_5
#define OLED_SCL_PIN GPIO_NUM_6
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_I2C_ADDR 0x3C

// Initialize the OLED display
esp_err_t oled_display_init(void);

// Clear the display
void oled_display_clear(void);

// Display text at specified position
void oled_display_text(uint8_t x, uint8_t y, const char* text);

// Display the recognized command
void oled_display_command(const char* command, uint8_t score);

// Update the display buffer
esp_err_t oled_display_update(void);

#ifdef __cplusplus
}
#endif

#endif // OLED_DISPLAY_H