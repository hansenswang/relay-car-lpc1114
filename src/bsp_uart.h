/**
 * @file    bsp_uart.h
 * @brief   UART0 console at 115200 8N1, retargeted for printf().
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include <stdint.h>

void    bsp_uart_init(uint32_t baud);
void    bsp_uart_putc(char c);
void    bsp_uart_puts(const char *s);
int     bsp_uart_getc_nonblocking(void);

#endif /* __BSP_UART_H__ */
