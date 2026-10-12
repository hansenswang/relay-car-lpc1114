/**
 * @file    bsp_oled.h
 * @brief   HS96L03W2C03 128x64 OLED over a 4-wire SPI-like bit-banged bus.
 */

#ifndef __BSP_OLED_H__
#define __BSP_OLED_H__

#include <stdint.h>

void bsp_oled_init(void);
void bsp_oled_clear(void);
void bsp_oled_show_string(uint8_t row, uint8_t col, const char *str);
void bsp_oled_printf(uint8_t row, uint8_t col, const char *fmt, ...);

#endif /* __BSP_OLED_H__ */
