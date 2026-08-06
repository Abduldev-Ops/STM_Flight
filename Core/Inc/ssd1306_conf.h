#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

#include "stm32l4xx_hal.h"

// Display size
#define SSD1306_128_64

// Use I2C (not SPI)
#define SSD1306_USE_I2C

// I2C port and address
#define SSD1306_I2C_PORT     hi2c1
#define SSD1306_I2C_ADDR     (0x3C << 1)

// Fonts to include
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18

#endif /* __SSD1306_CONF_H__ */
