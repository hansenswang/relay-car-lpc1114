/**
 * @file    bsp_key.h
 * @brief   Debounced start button.
 */

#ifndef __BSP_KEY_H__
#define __BSP_KEY_H__

#include <stdint.h>

void    bsp_key_init(void);

/** Poll from a periodic thread.  Returns 1 once per accepted press. */
uint8_t bsp_key_poll(void);

#endif /* __BSP_KEY_H__ */
