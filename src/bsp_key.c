/**
 * @file    bsp_key.c
 * @brief   Debounced start-button on P1_4 with a small state machine.
 *
 * The debounce is sample-counted rather than time-based: the UI thread calls
 * bsp_key_poll() every 100 ms, and a level must be seen on three consecutive
 * calls (300 ms) before it is accepted as a state change.  That is long
 * enough to swallow the ~10 ms of chatter the tactile switch produces, and
 * it costs one counter instead of a timer.
 */

#include "LPC11xx.h"
#include "bsp_key.h"

#define DEBOUNCE_COUNT   3

static uint8_t  stable_level  = 1;      /* idle high                        */
static uint8_t  last_level    = 1;
static uint8_t  agree_count   = 0;

extern volatile uint8_t key_flag;       /* consumed by the control thread   */

void bsp_key_init(void)
{
    LPC_GPIO1->DIR &= ~KEY_PIN;         /* input                            */
    LPC_GPIO1->IS  &= ~KEY_PIN;         /* edge sensitive (unused)          */
    stable_level = 1;
    last_level   = 1;
    agree_count  = 0;
}

/**
 * @brief   Poll the button; returns 1 exactly once per press.
 *
 * Call from a periodic thread.  The return value is a one-shot event, so a
 * caller that ignores it does not accumulate a phantom backlog.
 */
uint8_t bsp_key_poll(void)
{
    uint8_t level = (LPC_GPIO1->DATA & KEY_PIN) ? 1 : 0;

    if (level == last_level)
    {
        if (agree_count < DEBOUNCE_COUNT) { agree_count++; }
        if (agree_count == DEBOUNCE_COUNT && level != stable_level)
        {
            stable_level = level;
            if (stable_level == KEY_PRESSED)
            {
                key_flag = 1;
                return 1;
            }
        }
    }
    else
    {
        agree_count = 0;
        last_level  = level;
    }

    return 0;
}
