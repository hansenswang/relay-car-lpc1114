/**
 * @file    bsp_led.h
 * @brief   Status LED, debounced key input and software delays.
 *
 * K1 is read by the LED thread rather than by an interrupt: the start button
 * is pressed once at the beginning of a run, so a 100 ms poll with a simple
 * edge detect is both sufficient and immune to the mechanical chatter that
 * made an EXTI-based attempt trigger twice.
 */

#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#include <stdint.h>
#include "rtthread.h"

#define KEY_GPIO        LPC_GPIO1
#define KEY_PIN         (1UL << 4)      /* K1 on P1_4, active low        */
#define KEY_PRESSED     0

void     bsp_led_init(void);
void     bsp_led_on(void);
void     bsp_led_off(void);
void     bsp_led_toggle(void);

void     delay_us(uint32_t us);
void     delay_ms(uint32_t ms);

#endif /* __BSP_LED_H__ */
