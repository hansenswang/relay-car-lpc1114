/**
 * @file    pid.c
 * @brief   Positional PID with derivative-on-measurement and integral clamp.
 *
 * The controller is called every 10 ms from the core control thread, so the
 * derivative term is a plain backward difference of the measurement — no
 * explicit sample-time scaling is required, and fold the period into kd when
 * tuning.
 *
 * Two defences keep the output usable on a vehicle that can stall:
 *
 *   1. Anti-windup — the integral term is clamped to +/-integral_limit.  When
 *      the car is held against a wall the wheels cannot respond, the error
 *      stays large, and an unclamped integrator would saturate within ~1 s
 *      and then need a second to unwind.  The clamp bounds that recovery.
 *
 *   2. Output clamp — the final effort is bounded by +/-output_limit so a
 *      single bad sensor sample (a missing reflection, a stray mark) cannot
 *      command a full-speed pivot.
 */

#include "pid.h"

/* Tuning that was settled on during the December bench sessions.
 *   kp — dominant term; enough authority to pull back from a half-offset
 *        without oscillating on the straights.
 *   ki — small, and only retained so that a persistent mechanical bias
 *        (a dragging wheel) is eventually trimmed out.
 *   kd — damps the approach to the set point; roughly one tenth of kp.  */
#define PID_KP              8.0f
#define PID_KI              0.05f
#define PID_KD              0.60f

#define PID_INTEGRAL_LIMIT  20.0f
#define PID_OUTPUT_LIMIT    6.0f

void pid_init(PID_HandleTypeDef *pid)
{
    if (pid == 0) { return; }

    pid->kp             = PID_KP;
    pid->ki             = PID_KI;
    pid->kd             = PID_KD;
    pid->setpoint       = 0.0f;
    pid->integral       = 0.0f;
    pid->prev_error     = 0.0f;
    pid->output         = 0.0f;
    pid->integral_limit = PID_INTEGRAL_LIMIT;
    pid->output_limit   = PID_OUTPUT_LIMIT;
}

void pid_reset(PID_HandleTypeDef *pid)
{
    if (pid == 0) { return; }

    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

float pid_calculate(PID_HandleTypeDef *pid, float measurement)
{
    float error;

    if (pid == 0) { return 0.0f; }

    error = pid->setpoint - measurement;
    return pid_step(pid, error);
}

/**
 * @brief   Advance the controller by one sample using an explicit error.
 *
 * Split out from pid_calculate() so that callers which already know the
 * error (the line-following loop, which gets it for free from the sensor
 * decode) do not pay for a redundant subtraction.
 */
float pid_step(PID_HandleTypeDef *pid, float error)
{
    float derivative;
    float output;

    pid->integral += error;

    /* anti-windup */
    if (pid->integral >  pid->integral_limit) { pid->integral =  pid->integral_limit; }
    if (pid->integral < -pid->integral_limit) { pid->integral = -pid->integral_limit; }

    /* derivative on measurement — because error = setpoint - measurement and
     * setpoint is constant, d(error)/dt is the same as -d(measurement)/dt. */
    derivative = error - pid->prev_error;
    pid->prev_error = error;

    output = pid->kp * error
           + pid->ki * pid->integral
           + pid->kd * derivative;

    if (output >  pid->output_limit) { output =  pid->output_limit; }
    if (output < -pid->output_limit) { output = -pid->output_limit; }

    pid->output = output;
    return output;
}
