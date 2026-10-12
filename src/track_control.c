/**
 * @file    track_control.c
 * @brief   Line-sensor decode and differential steering.
 *
 * The five reflective channels are read as GPIO inputs on port 2, pins 7 to
 * 11.  Because the sensors are wired in ascending pin order, the five bits
 * land contiguously in the port data register and a single mask-and-shift
 * extracts them — no per-channel conditional reads, no debouncing loop.
 */

#include "LPC11xx.h"
#include "track_control.h"
#include "bsp_motor.h"
#include "pid.h"
#include <stdio.h>

/* ---- exported state --------------------------------------------------- */
volatile float    track_status[1];
volatile float    track_status_backup[1][20];
volatile uint32_t track_status_worse;
volatile _track_state track_state;

volatile float move_flag   = 0.0f;
volatile int   left_speed  = 0;
volatile int   right_speed = 0;

/* Handle for the steering compensator.  Defined here rather than in main so
 * that the control law travels with the code that produces its input.    */
PID_HandleTypeDef track_pid = {0};

#define BASE_SPEED   10.0f   /* straight-line cruise speed                */
#define TURN_SPEED    6.0f   /* ceiling while cornering                   */
#define TURN_SLIGHT   2.0f   /* step sizes for the fallback bang-bang law */
#define TURN_NORMAL   3.0f
#define TURN_FAST     4.0f
#define TURN_URGENT   5.0f

/* ---------------------------------------------------------------------- */

void track_init(void)
{
    /* The five sensor channels are plain digital inputs.  Configure the
     * direction register for P2[7..11] and leave the internal pulls off —
     * the ITR9909 boards drive the line actively.                      */
    LPC_GPIO2->DIR &= ~((1UL << 7) | (1UL << 8) | (1UL << 9) |
                        (1UL << 10) | (1UL << 11));

    track_status[0]  = 0.0f;
    track_status_worse = 0;
    track_state.raw  = 0;

    for (int i = 0; i < 20; i++)
    {
        track_status_backup[0][i] = 0.0f;
    }
}

/* ---------------------------------------------------------------------- */

uint8_t bsp_track_sample(void)
{
    uint32_t pins;
    uint8_t  mask;

    /* Read the whole port once, then extract bits 7..11 in one operation.
     * A single read matters: sampling the port five times in a row would
     * straddle a line edge and produce a mask that never existed.  */
    pins = LPC_GPIO2->DATA;
    mask = (uint8_t)((pins >> 7) & 0x1F);

    track_state.raw = mask;
    return mask;
}

/* ---------------------------------------------------------------------- */

/**
 * @brief   Translate a 5-bit line mask into a signed lateral offset.
 *
 * Rather than a 32-entry lookup table, the offset is computed from the
 * centre of gravity of the dark channels.  With channels numbered 0..4 the
 * centre of an all-white field is 2, so subtracting 2 shifts the neutral
 * point to zero.  A mask where only the leftmost channel is dark yields
 * (0 - 2) = -2, and so on.
 *
 * Normalising by the number of dark channels keeps the magnitude stable as
 * the car's nose yaws: a three-channel smear of the same line does not
 * produce three times the reported error.
 */
static float mask_to_offset(uint8_t mask)
{
    int   sum = 0;
    int   count = 0;
    float cog;

    if (mask == TRACK_ALL_WHITE || mask == TRACK_ALL_BLACK)
    {
        /* No usable directional information.  The state machine handles
         * these masks explicitly, so return the last known offset and
         * let the caller decide.  */
        return track_status[0];
    }

    for (int i = 0; i < 5; i++)
    {
        if (mask & (1u << i))
        {
            sum += i;
            count++;
        }
    }

    if (count == 0) { return track_status[0]; }

    cog = (float)sum / (float)count;    /* 0.0 .. 4.0 */
    return cog - 2.0f;                  /* -2.0 .. +2.0 */
}

/* ---------------------------------------------------------------------- */

void track_check(void)
{
    /* bsp_track_sample() has already latched track_state.raw this cycle. */
    track_status[0] = mask_to_offset(track_state.raw);

    /* "Worsening" heuristic — see the header for why this exists.  It is
     * advisory only; nothing in the control path acts on it, but it is
     * shown on the OLED and was the trigger we used while tuning.      */
    for (int i = 19; i > 0; i--)
    {
        track_status_backup[0][i] = track_status_backup[0][i - 1];
    }
    track_status_backup[0][0] = track_status[0];

    if ((track_status[0] > 0 && track_status_backup[0][1] > 0 &&
         track_status[0] > track_status_backup[0][1]) ||
        (track_status[0] < 0 && track_status_backup[0][1] < 0 &&
         track_status[0] < track_status_backup[0][1]))
    {
        track_status_worse++;
    }
    else
    {
        track_status_worse = 0;
    }
}

/* ---------------------------------------------------------------------- */

/**
 * @brief   Compute the wheel commands from the current offset.
 *
 * Two control laws live here.  The one that shipped is the PID path; the
 * commented-out switch is the bang-bang law that was tried first.  Keeping
 * both documents the comparison, and the reasons are recorded in
 * results/README.md — briefly, PID was smooth on the straights but needed
 * retuning at every speed, while the six-step law was trivially predictable
 * but stepped visibly and could not hold a straight line.
 */
void track_move_control(void)
{
    /* Clamp the offset to the range the sensor geometry can actually
     * produce.  Anything outside +/-4 is a misread — typically one or two
     * channels seeing the reflective floor — and acting on it would send
     * the car into a hard pivot.  */
    move_flag = track_status[0];
    if (move_flag >  4.0f) { move_flag =  4.0f; }
    if (move_flag < -4.0f) { move_flag = -4.0f; }

    /* Dead band: within +/-0.3 both channels are effectively on the line,
     * and driving the integrator here only introduces noise.  Clearing the
     * integral (rather than merely freezing it) is what stops the small
     * residual wind-up that used to make the car creep sideways at rest. */
    if (move_flag > -0.3f && move_flag < 0.3f)
    {
        move_flag = 0.0f;
        track_pid.integral = 0.0f;
        track_pid.output   = 0.0f;
    }

    pid_step(&track_pid, move_flag);

    /* Differential split: the correction is subtracted from one wheel and
     * added to the other, so the average speed stays at BASE_SPEED and the
     * steering response is symmetric.
     *
     * Positive offset (line to the left) => left wheel slows, right speeds
     * up => the car yaws left.  Negative offset mirrors it.             */
    left_speed  = (int)(BASE_SPEED - track_pid.output);
    right_speed = (int)(BASE_SPEED + track_pid.output);

    /* ---- fallback: six-step bang-bang steering (not compiled) ---------
     *
     * int flag = (int)move_flag;
     * switch (flag)
     * {
     *     case  0: left_motor(BASE_SPEED);                 right_motor(BASE_SPEED);                 break;
     *     case  1: left_motor(BASE_SPEED - TURN_SLIGHT);   right_motor(BASE_SPEED + TURN_SLIGHT);   break;
     *     case  2: left_motor(BASE_SPEED - TURN_NORMAL);   right_motor(BASE_SPEED + TURN_NORMAL);   break;
     *     case  3: left_motor(BASE_SPEED - TURN_FAST);     right_motor(BASE_SPEED + TURN_FAST);     break;
     *     case  4: left_motor(BASE_SPEED - TURN_URGENT);   right_motor(BASE_SPEED + TURN_URGENT);   break;
     *     case -1: left_motor(BASE_SPEED + TURN_SLIGHT);   right_motor(BASE_SPEED - TURN_SLIGHT);   break;
     *     case -2: left_motor(BASE_SPEED + TURN_NORMAL);   right_motor(BASE_SPEED - TURN_NORMAL);   break;
     *     case -3: left_motor(BASE_SPEED + TURN_FAST);     right_motor(BASE_SPEED - TURN_FAST);     break;
     *     case -4: left_motor(BASE_SPEED + TURN_URGENT);   right_motor(BASE_SPEED - TURN_URGENT);   break;
     *     default: left_motor(BASE_SPEED);                 right_motor(BASE_SPEED);                 break;
     * }
     * ------------------------------------------------------------------- */
}
