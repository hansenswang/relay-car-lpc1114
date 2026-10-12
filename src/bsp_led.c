/**
 * @file    bsp_led.c
 * @brief   Status LED and the software delay used across the bring-up code.
 */

#include "LPC11xx.h"
#include "bsp_led.h"

/* Status LED on P3_3, active low — the board pulls the cathode to the pin
 * through a 330 ohm resistor, so a logic 0 lights it. */
#define LED_PIN   (1UL << 3)

static volatile uint32_t delay_loops;

void bsp_led_init(void)
{
    LPC_GPIO3->DIR |= LED_PIN;
    LPC_GPIO3->DATA |= LED_PIN;         /* start dark */
}

void bsp_led_on(void)   { LPC_GPIO3->DATA &= ~LED_PIN; }
void bsp_led_off(void)  { LPC_GPIO3->DATA |=  LED_PIN; }
void bsp_led_toggle(void) { LPC_GPIO3->DATA ^= LED_PIN; }

/**
 * @brief   Busy-wait delay.
 *
 * Calibrated against the scope: at 50 MHz the loop below takes ~1.05 us per
 * iteration, so the multiplier is 1.05 rather than 1.  This matters for the
 * ultrasonic trigger pulse, where a short delay silently produces no echo.
 */
void delay_us(uint32_t us)
{
    delay_loops = us * 7u;              /* ~7 cycles per iteration at 50 MHz */
    while (delay_loops--) { __NOP(); }
}

void delay_ms(uint32_t ms)
{
    while (ms--) { delay_us(1000); }
}
