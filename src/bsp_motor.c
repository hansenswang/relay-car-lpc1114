/**
 * @file    bsp_motor.c
 * @brief   RZ7899 motor-driver interface with soft-start ramping.
 *
 * Wiring
 * ------
 *   left  motors : IN1 = P0_8,  IN2 = P0_1,  EN = P0_9   (paralleled pair)
 *   right motors : IN1 = P1_9,  IN2 = P1_1,  EN = P1_10  (paralleled pair)
 *   brake        : all six of the above driven high simultaneously
 *
 * The enable pin of each RZ7899 is fed from a software PWM.  A hardware
 * timer would be cleaner, but the two timers that could do this are already
 * committed (CT16B0 to the RT-Thread tick, CT16B1 to the ultrasonic
 * capture), so the 10 ms control loop halves as the PWM source: the duty is
 * expressed as a fraction of that 10 ms window.
 */

#include "LPC11xx.h"
#include "bsp_motor.h"
#include "rtthread.h"

/* ---- pin masks -------------------------------------------------------- */
#define L_IN1   (1UL << 8)      /* P0_8  */
#define L_IN2   (1UL << 1)      /* P0_1  */
#define L_EN    (1UL << 9)      /* P0_9  */

#define R_IN1   (1UL << 9)      /* P1_9  */
#define R_IN2   (1UL << 1)      /* P1_1  */
#define R_EN    (1UL << 10)     /* P1_10 */

/* ---- soft-start profile ----------------------------------------------- */
/* A step change from 0 to full duty makes the N20 gearbox chatter and, on
 * the vinyl surface, bounce the front wheels off the ground.  Ramping the
 * commanded magnitude by this much per control period removes the chatter
 * entirely at the cost of about 30 ms of extra spin-up.                  */
#define RAMP_STEP   4

static int ramp_left  = 0;
static int ramp_right = 0;

/* ---------------------------------------------------------------------- */

void bsp_motor_init(void)
{
    /* direction + enable pins on port 0 */
    LPC_GPIO0->DIR |= L_IN1 | L_IN2 | L_EN;
    /* direction + enable pins on port 1 */
    LPC_GPIO1->DIR |= R_IN1 | R_IN2 | R_EN;

    bsp_motor_stop();

    ramp_left  = 0;
    ramp_right = 0;
}

/* ---------------------------------------------------------------------- */

void bsp_motor_stop(void)
{
    /* Both inputs low and enable low = coast to a stop.  Driving the two
     * inputs high instead would apply a short across the motor (braking);
     * that is available via brake() below but is not what a stop command
     * should do — a hard brake at speed made the car spin on the spot.  */
    LPC_GPIO0->DATA &= ~(L_IN1 | L_IN2 | L_EN);
    LPC_GPIO1->DATA &= ~(R_IN1 | R_IN2 | R_EN);
}

/**
 * @brief   Short both motor windings — dynamic braking.
 *
 * Retained because it is the correct response to the "obstacle within 40 cm"
 * condition.  It is deliberately not wired to any state in the line-following
 * run: the relay task has no obstacle, and an accidental brake mid-lap was
 * one of the failure modes we saw.
 */
void car_brake(void)
{
    LPC_GPIO0->DATA |= L_IN1 | L_IN2 | L_EN;
    LPC_GPIO1->DATA |= R_IN1 | R_IN2 | R_EN;
}

/* ---------------------------------------------------------------------- */

/**
 * @brief   Drive one side.
 *
 * @param in1_mask   direction pin A mask
 * @param in2_mask   direction pin B mask
 * @param en_mask    enable pin mask
 * @param port0      non-zero if the pins live on port 0, else port 1
 * @param speed      -100 .. +100
 */
static void drive_side(uint32_t in1_mask, uint32_t in2_mask, uint32_t en_mask,
                       int port0, int speed)
{
    volatile uint32_t *reg = port0 ? &LPC_GPIO0->DATA : &LPC_GPIO1->DATA;
    uint32_t magnitude;
    uint32_t on_time_us;

    if (speed >  100) { speed =  100; }
    if (speed < -100) { speed = -100; }

    /* direction */
    if (speed >= 0)
    {
        *reg = (*reg & ~in1_mask) | in2_mask;   /* forward */
    }
    else
    {
        *reg = (*reg & ~in2_mask) | in1_mask;   /* reverse */
    }

    magnitude = (uint32_t)((speed >= 0) ? speed : -speed);

    /* Duty is realised by holding the enable pin high for on_time_us out of
     * the 10 ms control window.  At the magnitudes the car actually uses
     * (single digits out of 100) this is a few hundred microseconds, well
     * below the point where the PWM period starts to be audible.        */
    on_time_us = magnitude * 100;        /* 100 % => 10 000 us */

    if (on_time_us == 0)
    {
        *reg &= ~en_mask;
        return;
    }

    *reg |= en_mask;
    rt_hw_us_delay(on_time_us);
    *reg &= ~en_mask;
}

/* ---------------------------------------------------------------------- */

void bsp_motor_set(int left, int right)
{
    /* Apply the ramp toward the requested magnitude. */
    if (left > ramp_left)       { ramp_left  += RAMP_STEP; if (ramp_left  > left)  ramp_left  = left;  }
    else if (left < ramp_left)  { ramp_left  -= RAMP_STEP; if (ramp_left  < left)  ramp_left  = left;  }

    if (right > ramp_right)     { ramp_right += RAMP_STEP; if (ramp_right > right) ramp_right = right; }
    else if (right < ramp_right){ ramp_right -= RAMP_STEP; if (ramp_right < right) ramp_right = right; }

    drive_side(L_IN1, L_IN2, L_EN, 1, ramp_left);
    drive_side(R_IN1, R_IN2, R_EN, 0, ramp_right);
}

/* ---------------------------------------------------------------------- */

void left_motor(int speed)
{
    bsp_motor_set(speed, ramp_right);
}

void right_motor(int speed)
{
    bsp_motor_set(ramp_left, speed);
}

void left_motor_dir(uint8_t dir, uint8_t duty)
{
    left_motor((dir == MOTOR_FORWARD) ? (int)duty : -(int)duty);
}

void right_motor_dir(uint8_t dir, uint8_t duty)
{
    right_motor((dir == MOTOR_FORWARD) ? (int)duty : -(int)duty);
}
