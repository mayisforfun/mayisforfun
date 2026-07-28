#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdint.h>

/*
 * Q10 fixed-point speed PID controller.
 * target and actual must use the same unit. In main.c they are encoder ticks
 * per 10 ms control cycle. Output is a signed PWM correction.
 */
typedef struct {
    int16_t kp_q10;      /* P gain in Q10 format */
    int16_t ki_q10;      /* I gain in Q10 format */
    int16_t kd_q10;      /* D gain in Q10 format */
    int32_t integral;    /* accumulated error, reset when enabling */
    int16_t last_error;  /* previous cycle error for D term */
    int16_t max_output;  /* correction clamp, in PWM counts */
} SpeedPID;

void     SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                       int16_t kd_q10, int16_t max_output);
int16_t  SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual);
void     SpeedPID_reset(SpeedPID *pid);

/* Legacy helper. The current closed-loop code uses SPEED_TARGET_TICKS_PER_1000
 * in main.c so it is easy to tune beside the line parameters.
 */
#define SPEED_TICKS_PER_1000  55

static inline int16_t Encoder_ticksToSpeed(int32_t ticks)
{
    return (int16_t)((ticks * 1000) / SPEED_TICKS_PER_1000);
}

#endif
