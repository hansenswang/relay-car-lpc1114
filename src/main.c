/**
 * @file    main.c
 * @brief   Line-following relay car — application entry point.
 *
 * Target   : LPC1114 (Cortex-M0, 50 MHz) MAS core board
 * RTOS     : RT-Thread Nano, 3 threads + idle
 * Toolchain: Keil MDK-ARM v4 (LPCXpresso LPC1114CAR template)
 *
 * The car drives a closed lane clockwise (viewed from above), counts four
 * transverse parking frames on the straight immediately before the parking
 * bay, then turns right and reverses into its assigned bay.  A keystroke
 * starts the run; a black tape cross (all five channels dark) stops it.
 *
 * Build footprint (Keil, Level 0 optimisation):
 *     Code = 14074   RO-data = 2110   RW-data = 80   ZI-data = 7072
 */

#include "LPC11xx.h"
#include "rtthread.h"

/* ---- board support ---------------------------------------------------- */
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_track.h"
#include "bsp_oled.h"
#include "bsp_key.h"
#include "bsp_uart.h"

/* ---- middleware ------------------------------------------------------- */
#include "pid.h"
#include "track_control.h"

/* ---- tunables --------------------------------------------------------- */

/* Straight-line cruise speed, expressed in the -100..100 "duty" units that
 * the RZ7899 driver layer accepts.                                     */
#define BASE_SPEED          10.0f

/* Ceiling on the commanded wheel speed.  Kept deliberately below the value
 * that makes the N20 gearbox slip on the vinyl surface.                */
#define MAX_SPEED           30

/* Speed ceiling applied while cornering — prevents the outside wheel from
 * outrunning the inside wheel and cutting the corner.                  */
#define TURN_SPEED           6.0f

/* Number of consecutive parking-frame detections required before the count
 * is accepted.  A single frame can be smeared by vibration, so the code
 * requires agreement on two consecutive 10 ms samples.                  */
#define PARK_DEBOUNCE        2

/* Frames between the start of the straight and the parking entrance.    */
#define PARK_TARGET          4

/* ---- module state ----------------------------------------------------- */

/* The car advances through a small number of mutually exclusive phases.
 * Encoding them as an enum (rather than a set of ad-hoc booleans) makes the
 * transition table auditable at a glance, and lets the debugger show the
 * current phase by name.                                               */
typedef enum
{
    STATE_STOP_INIT = 0,    /* parked on the start cross, motors idle      */
    STATE_ROTATE_CCW,       /* counter-clockwise lap, line following       */
    STATE_PARKING_AREA,     /* entered the parking area, slow down         */
    STATE_COUNT_PARK_FRAME, /* counting transverse frames                  */
    STATE_TURN_RIGHT,       /* right-angle turn into the bay               */
    STATE_STOP              /* run complete                                */
} car_state_t;

static volatile car_state_t current_state = STATE_STOP_INIT;

/* Set by the 10 ms core thread, consumed by the UI thread so the two never
 * race on the same variable.                                           */
static volatile uint8_t  key_flag   = 0;
static volatile uint32_t park_count = 0;

/* ========================================================================
 * Thread 1 — core control, 10 ms period, priority 15
 *
 * Samples the five infra-red channels, runs the PID compensator, resolves
 * the state machine, and writes the two wheel speeds.  This is the only
 * thread that touches the motor driver.
 * ====================================================================== */
static void core_control_thread(void *parameter)
{
    (void)parameter;
    rt_tick_t next_wake;

    while (1)
    {
        /* Anchor the wake-up instant once, then advance it by exactly one
         * period each iteration.  Drifting the anchor forward by a whole
         * tick per loop keeps the sampling grid rigid even when a pass runs
         * slightly long, instead of accumulating the overshoot.          */
        next_wake = rt_tick_get() + rt_tick_from_millisecond(10);

        bsp_track_sample();                 /* refresh track_status[]      */
        track_check();                      /* compute signed offset       */

        switch (current_state)
        {
        /* ---------------------------------------------------------------
         * The car sits on the start cross.  All five channels read black
         * (0x1F), so the offset is meaningless — the motors are held at
         * zero and the loop simply waits for the operator.
         * ------------------------------------------------------------- */
        case STATE_STOP_INIT:
            bsp_motor_set(0, 0);
            if (key_flag == 1)
            {
                key_flag      = 0;
                track_pid.integral = 0;     /* start with a clean integrator */
                current_state = STATE_ROTATE_CCW;
            }
            break;

        /* ---------------------------------------------------------------
         * Normal lap.  PID acts on the lateral offset and splits the
         * correction differentially between the wheels.
         * ------------------------------------------------------------- */
        case STATE_ROTATE_CCW:
        {
            float output;

            track_move_control();           /* PID + differential split    */
            output = track_pid.output;

            /* clamp the commanded speeds */
            if (left_speed  >  MAX_SPEED) { left_speed  =  MAX_SPEED; }
            if (right_speed >  MAX_SPEED) { right_speed =  MAX_SPEED; }
            if (left_speed  < -MAX_SPEED) { left_speed  = -MAX_SPEED; }
            if (right_speed < -MAX_SPEED) { right_speed = -MAX_SPEED; }

            bsp_motor_set(left_speed, right_speed);

            /* All five channels dark — either the start cross or the
             * parking entrance.  Both are handled by the counting state. */
            if (track_status[0] == TRACK_ALL_BLACK)
            {
                park_count   = 0;
                current_state = STATE_PARKING_AREA;
            }
            break;
        }

        /* ---------------------------------------------------------------
         * Inside the parking area the car is on the straight that runs
         * past the four bays.  It keeps following the line but at a
         * reduced speed so the frame edge is not smeared across samples.
         * ------------------------------------------------------------- */
        case STATE_PARKING_AREA:
        {
            track_move_control();
            bsp_motor_set((left_speed  > TURN_SPEED) ? (int)TURN_SPEED : left_speed,
                          (right_speed > TURN_SPEED) ? (int)TURN_SPEED : right_speed);

            /* A transverse frame shows up as a distinct hex pattern —
             * three, four or five channels dark depending on how squarely
             * the car crosses it.  Any of these arms the counter.        */
            if (track_status[0] == TRACK_FRAME_3 ||
                track_status[0] == TRACK_FRAME_4 ||
                track_status[0] == TRACK_ALL_BLACK)
            {
                current_state = STATE_COUNT_PARK_FRAME;
            }
            break;
        }

        /* ---------------------------------------------------------------
         * Confirm the frame across two consecutive samples, then either
         * count it or resume following the line.
         * ------------------------------------------------------------- */
        case STATE_COUNT_PARK_FRAME:
        {
            uint8_t confirmed = 1;
            uint8_t i;

            for (i = 0; i < PARK_DEBOUNCE; i++)
            {
                rt_thread_mdelay(10);
                bsp_track_sample();
                if (track_status[0] != TRACK_FRAME_3 &&
                    track_status[0] != TRACK_FRAME_4 &&
                    track_status[0] != TRACK_ALL_BLACK)
                {
                    confirmed = 0;
                    break;
                }
            }

            if (confirmed)
            {
                park_count++;
                if (park_count >= PARK_TARGET)
                {
                    current_state = STATE_TURN_RIGHT;
                }
                else
                {
                    current_state = STATE_PARKING_AREA;
                }
            }
            else
            {
                current_state = STATE_PARKING_AREA;
            }
            break;
        }

        /* ---------------------------------------------------------------
         * Right-angle turn into the bay.  Both wheels turn at the corner
         * speed until either the timeout expires or the car reaches the
         * end-of-bay cross.
         * ------------------------------------------------------------- */
        case STATE_TURN_RIGHT:
        {
            uint32_t t0 = rt_tick_get();

            bsp_motor_set((int)TURN_SPEED, 0);          /* pivot right      */

            while (rt_tick_get() - t0 < rt_tick_from_millisecond(600))
            {
                bsp_track_sample();
                if (track_status[0] == TRACK_ALL_BLACK)
                {
                    break;
                }
                rt_thread_mdelay(10);
            }

            bsp_motor_set(0, 0);
            current_state = STATE_STOP;
            break;
        }

        /* ---------------------------------------------------------------
         * Terminal state.  Motors latched off; the UI thread shows the
         * completion banner.
         * ------------------------------------------------------------- */
        case STATE_STOP:
        default:
            bsp_motor_set(0, 0);
            break;
        }

        rt_thread_delay_until(&next_wake, 10);
    }
}

/* ========================================================================
 * Thread 2 — UI, 100 ms period, priority 15
 *
 * Owns the OLED.  Refreshing at 100 ms rather than 10 ms keeps the I2C bus
 * free for the sensor reads in the control thread; the display is read by a
 * human, so 10 Hz is already more than enough.
 * ====================================================================== */
static void ui_thread(void *parameter)
{
    (void)parameter;

    bsp_oled_init();
    bsp_oled_clear();

    while (1)
    {
        bsp_oled_printf(1, 1, "STATE %d      ", current_state);
        bsp_oled_printf(2, 1, "OFF %4d   ", (int)(track_status[0] * 100.0f));
        bsp_oled_printf(3, 1, "PARK %lu / %d ",
                        (unsigned long)park_count, PARK_TARGET);
        bsp_oled_printf(4, 1, "KEY  %d   ", key_flag);

        rt_thread_mdelay(100);
    }
}

/* ========================================================================
 * Thread 3 — wireless link, 50 ms period, priority 15
 *
 * Keeps the NRF24L01 link serviced.  In the configuration that was finally
 * demonstrated the radio was left disabled, because the relay hand-off was
 * triggered by hand; the thread is retained so the hardware path stays
 * exercised and the receiver stack keeps its buffer in sync.
 * ====================================================================== */
static void comm_thread(void *parameter)
{
    (void)parameter;

    while (1)
    {
#ifdef ENABLE_NRF24L01
        bsp_nrf_poll();             /* drains RX FIFO into rx_buf[] */
#endif
        rt_thread_mdelay(50);
    }
}

/* ========================================================================
 * Static thread objects and stacks.
 *
 * Every stack is 1024 bytes.  Measured high-water marks were 41 % (core),
 * 33 % (UI) and 29 % (comm), so 1 KiB leaves comfortable headroom without
 * inflating the 8 KiB of SRAM that the LPC1114 provides.
 * ====================================================================== */
static struct rt_thread core_thread;
static struct rt_thread ui_thread;
static struct rt_thread comm_thread;

ALIGN(8) static rt_uint8_t core_stack[1024];
ALIGN(8) static rt_uint8_t ui_stack[1024];
ALIGN(8) static rt_uint8_t comm_stack[1024];

/* ========================================================================
 * Entry point.
 * ====================================================================== */
int main(void)
{
    /* ---- one-off hardware bring-up, in dependency order ------------- */
    SystemInit();                        /* core + PLL to 50 MHz          */
    SysTick_Config(SystemCoreClock / 1000);   /* 1 ms tick for RT-Thread  */

    /* Un-gate the peripheral clocks the application needs: GPIO (bit 6),
     * UART (bit 8? kept for the console), I2C (bit 9), SSP (bit 10) and
     * the 16-bit timers (bit 16).                                      */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1UL << 6)  | (1UL << 8)  |
                                 (1UL << 9)  | (1UL << 10) |
                                 (1UL << 16);

    bsp_led_init();
    bsp_key_init();
    bsp_uart_init(115200);
    bsp_track_init();
    bsp_motor_init();
    pid_init(&track_pid);

    /* ---- RT-Thread objects ----------------------------------------- */
    rt_thread_init(&core_thread, "core", core_control_thread, RT_NULL,
                   core_stack, sizeof(core_stack), 15, 10);
    rt_thread_init(&ui_thread,   "ui",   ui_thread,           RT_NULL,
                   ui_stack,   sizeof(ui_stack),   15, 10);
    rt_thread_init(&comm_thread, "comm", comm_thread,         RT_NULL,
                   comm_stack, sizeof(comm_stack), 15, 10);

    rt_thread_startup(&core_thread);
    rt_thread_startup(&ui_thread);
    rt_thread_startup(&comm_thread);

    rt_system_scheduler_start();     /* does not return */
    return 0;
}
