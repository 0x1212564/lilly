#ifndef ST7735_DRIVER_H_
#define ST7735_DRIVER_H_

#include <avr/io.h>

/**
 * @file st7735_driver.h
 * @brief Declaraties voor de ST7735 display driver.
 */

void ST7735_init(void);
void ST7735_draw_image_from_progmem(const uint16_t* img_data);

#endif // ST7735_DRIVER_H_