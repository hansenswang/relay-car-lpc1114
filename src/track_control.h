/**
 * @file    track_control.h
 * @brief   Line-sensor decoding, offset calculation and differential steering.
 *
 * Five reflective optical sensors (ITR9909) sit in a row under the nose,
 * spaced 12 mm apart.  Each channel reports 0 when it sees white paper and 1
 * when it sees the black line, so one 10 ms sample collapses to a 5-bit mask.
 * The mask is decoded into a signed lateral offset, which is then handed to
 * the PID controller.
 */

#ifndef __TRACK_CONTROL_H__
#define __TRACK_CONTROL_H__

#include <stdint.h>

/* ---- raw mask values -------------------------------------------------- */

#define TRACK_LEFT_2        0x01    /* outermost left   channel            */
#define TRACK_LEFT_1        0x02
#define TRACK_CENTRE        0x04
#define TRACK_RIGHT_1       0x08
#define TRACK_RIGHT_2       0x10    /* outermost right  channel            */

#define TRACK_ALL_BLACK     0x1F    /* cross / start / end-of-bay marker   */
#define TRACK_ALL_WHITE     0x00    /* lost the line                       */

/* A transverse parking frame is 25 mm wide, so the car normally sees three
 * or four channels dark as it crosses.  Both patterns are accepted; a frame
 * seen edge-on can also momentarily read as all-black, which is covered by
 * the TRACK_ALL_BLACK case in the state machine.                        */
#define TRACK_FRAME_3       0x1C    /* channels 2,3,4 dark                 */
#define TRACK_FRAME_4       0x18    /* channels 3,4 dark                   */

/* ---- shared state ----------------------------------------------------- */

/* Signed lateral offset, in "channel units":
 *     negative — the car is right of the line and must steer left
 *     positive — the car is left  of the line and must steer right
 *     magnitude 0..4 maps one-to-one onto how far off centre the line is. */
extern volatile float track_status[1];

/* Rolling history kept for the "worsening" heuristic: if the offset keeps
 * growing over 20 samples while the car is commanding a correction, the car
 * is off the track rather than merely correcting.                        */
extern volatile float track_status_backup[1][20];
extern volatile uint32_t track_status_worse;

/* Legacy union retained so the original struct-based readers still compile. */
typedef union
{
    uint8_t  raw;
    struct
    {
        uint8_t position : 4;
        uint8_t valid    : 1;
    } bits;
} _track_state;

extern volatile _track_state track_state;

/* ---- API -------------------------------------------------------------- */

void track_init(void);

/** Sample the five channels and latch the raw mask. */
uint8_t bsp_track_sample(void);

/** Decode the latched mask into track_status[0] (the signed offset). */
void track_check(void);

/** Run the PID controller on the current offset, then split the result
 *  differentially between the two wheels into left_speed / right_speed. */
void track_move_control(void);

#endif /* __TRACK_CONTROL_H__ */
