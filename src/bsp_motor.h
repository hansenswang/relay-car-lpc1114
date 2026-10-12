/**
 * @file    bsp_motor.h
 * @brief   Four-wheel drive, RZ7899 half-bridge drivers, PWM speed control.
 *
 * Each RZ7899 channel takes two logic inputs (IN1/IN2) for direction and a
 * third pin for the enable/PWM input.  On this board the two motors on each
 * side are wired in parallel, so the firmware sees a two-channel drive:
 * left and right.
 */

#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include <stdint.h>

#define MOTOR_FORWARD    1
#define MOTOR_REVERSE    0

/** Configure the motor GPIO and the PWM timebase. */
void bsp_motor_init(void);

/** Set the two wheel speeds, -100 (full reverse) .. +100 (full forward). */
void bsp_motor_set(int left, int right);

/** Single-wheel forms, used by the bring-up test in results/README.md. */
void left_motor(int speed);
void right_motor(int speed);

/** Direction-explicit form kept for the original call sites. */
void left_motor_dir(uint8_t dir, uint8_t duty);
void right_motor_dir(uint8_t dir, uint8_t duty);

void bsp_motor_stop(void);

#endif /* __BSP_MOTOR_H__ */
