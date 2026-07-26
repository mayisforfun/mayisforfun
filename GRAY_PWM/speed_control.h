#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdint.h>

/*
 * Speed PID controller — Q10 fixed-point, same style as line_follow PID.
 * One SpeedPID instance per motor (left / right).
 *
 * target and actual are both in the [0..1000] speed-unit range.
 * The PID output is a signed correction added to the target.
 */
typedef struct {
    int16_t kp_q10;
    int16_t ki_q10;
    int16_t kd_q10;
    int32_t integral;
    int16_t last_error;
    int16_t max_output;
} SpeedPID;

void     SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                       int16_t kd_q10, int16_t max_output);
int16_t  SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual);
void     SpeedPID_reset(SpeedPID *pid);

/*
 * Calibration constant — how many encoder ticks per control cycle
 * correspond to a speed value of 1000.
 *
 * Example: at max PWM the wheel spins ~300 RPM with an 11-PPR encoder.
 *   300 RPM = 5 rev/s → 5 × 11 = 55 pulses/s.
 *   At 100 Hz control loop → 0.55 pulses/cycle × 1000 = ~550 ticks/cycle at speed 1000.
 *
 * This constant MUST be measured on the real robot.
 *   1. Run motor at speed = 1000 (PWM 100%).
 *   2. Read Encoder_getTicks() over 10 control cycles.
 *   3. Average ticks per cycle → that's SPEED_TICKS_PER_1000.
 *
 * Default: 55 (placeholder, adjust after measurement)
 */
#define SPEED_TICKS_PER_1000  55

static inline int16_t Encoder_ticksToSpeed(int32_t ticks)
{
    return (int16_t)((ticks * 1000) / SPEED_TICKS_PER_1000);
}

#endif
