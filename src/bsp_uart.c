/**
 * @file    bsp_uart.c
 * @brief   UART0 console, 115200 8N1, with printf() retargeting.
 *
 * Clocking
 * --------
 * The LPC1114 routes the UART a fractional divider (FDR) between the
 * peripheral clock and the baud generator, which is how a non-integer
 * division is reached.  With a 48 MHz PCLK and a UARTCLKDIV of 4 the input
 * to the baud generator is 12 MHz.  To land on 115200 the divisor would have
 * to be 104.1667, so the fractional divider is used to push the effective
 * input up to 12.5 MHz, giving a divisor of 4 (see the table below).
 *
 *   PCLK            48 MHz
 *   UARTCLKDIV      4          -> 12 MHz
 *   FDR             0x85       -> FR 1.627, DIVADDVAL 5, MULVAL 8
 *   DLM/DLL         0 / 4      -> divisor 4
 *   effective baud  115384     -> 0.16 % high, well inside the ~2 % budget
 *
 * The 0.16 % error is worth stating explicitly: it is three orders of
 * magnitude below the tolerance of the CH340 adapter that was eventually
 * used, and it never produced a framing error in any of the bench sessions.
 */

#include "LPC11xx.h"
#include "bsp_uart.h"
#include <stdio.h>

#define UARTCLKDIV_VALUE    4
#define UART_FDR_VALUE      0x85        /* MULVAL 8 | DIVADDVAL 5       */
#define UART_DLL_VALUE      4
#define UART_DLM_VALUE      0

void bsp_uart_init(uint32_t baud)
{
    uint32_t fdr;

    (void)baud;     /* the register set below is fixed to 115200 */

    /* Route P0_1 and P0_2? — on this board UART0 is on P1_6 (TXD) and
     * P1_7 (RXD), brought out to the 4-pin header.  Set the IOCON function
     * to 1 (TXD/RXD) for both.                                          */
    LPC_IOCON->PIO1_6 = 0x01;
    LPC_IOCON->PIO1_7 = 0x01;

    /* peripheral clock divider */
    LPC_SYSCON->UARTCLKDIV = UARTCLKDIV_VALUE;

    LPC_UART->LCR = 0x83;           /* 8N1, divisor latch access enabled */

    fdr = UART_FDR_VALUE;
    LPC_UART->FDR = fdr;

    LPC_UART->DLM = UART_DLM_VALUE;
    LPC_UART->DLL = UART_DLL_VALUE;

    LPC_UART->LCR = 0x03;           /* 8N1, divisor latch locked */
    LPC_UART->FCR = 0x07;           /* enable + reset both FIFOs  */
    LPC_UART->IER = 0x00;           /* polling only for now       */
}

void bsp_uart_putc(char c)
{
    while (!(LPC_UART->LSR & 0x20)) { }   /* wait for THR empty */
    LPC_UART->THR = (uint8_t)c;
}

void bsp_uart_puts(const char *s)
{
    while (*s) { bsp_uart_putc(*s++); }
}

int bsp_uart_getc_nonblocking(void)
{
    if (LPC_UART->LSR & 0x01)
    {
        return (int)(LPC_UART->RBR & 0xFF);
    }
    return -1;
}

/* ------------------------------------------------------------------------
 * printf() retargeting.
 *
 * Keil's microlib routes printf through fputc(), so overriding it sends the
 * formatted output to UART0.  The '\n' translation is applied here rather
 * than at every call site because a bare LF on a serial terminal returns the
 * carriage without moving to column 0 — the classic "staircase" output.
 * ---------------------------------------------------------------------- */
int fputc(int ch, FILE *f)
{
    (void)f;

    if (ch == '\n')
    {
        bsp_uart_putc('\r');
    }
    bsp_uart_putc((char)ch);
    return ch;
}
