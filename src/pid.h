/**
 * @file    pid.h
 * @brief   Minimal positional PID controller used for line-following steering.
 */

#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct
{
    float kp;               /* proportional gain                          */
    float ki;               /* integral gain                              */
    float kd;               /* derivative gain                            */
    float setpoint;         /* target value (0 — stay on the centre line) */
    float integral;         /* accumulated error                          */
    float prev_error;       /* error from the previous call               */
    float output;           /* last computed control effort               */
    float integral_limit;   /* anti-windup clamp on |integral|            */
    float output_limit;     /* clamp on |output|                          */
} PID_HandleTypeDef;

void  pid_init(PID_HandleTypeDef *pid);
float pid_calculate(PID_HandleTypeDef *pid, float measurement);
float pid_step(PID_HandleTypeDef *pid, float error);
void  pid_reset(PID_HandleTypeDef *pid);

#endif /* __PID_H__ */
