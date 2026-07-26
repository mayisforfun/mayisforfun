#include "speed_control.h"

void SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                   int16_t kd_q10, int16_t max_output)
{
    pid->kp_q10     = kp_q10;
    pid->ki_q10     = ki_q10;
    pid->kd_q10     = kd_q10;
    pid->max_output = max_output;
    pid->integral   = 0;
    pid->last_error = 0;
}

int16_t SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual)
{
    int16_t error = target - actual;
    int16_t derivative = error - pid->last_error;
    int32_t correction;
    int32_t i_term = 0;

    pid->last_error = error;

    if (pid->ki_q10 != 0) {
        int32_t max_integral = ((int32_t)pid->max_output * 1024) / pid->ki_q10;
        if (max_integral < 100) {
            max_integral = 100;
        }

        pid->integral += error;
        if (pid->integral > max_integral) {
            pid->integral = max_integral;
        } else if (pid->integral < -max_integral) {
            pid->integral = -max_integral;
        }

        i_term = (int32_t)pid->ki_q10 * pid->integral;
    } else {
        pid->integral = 0;
    }

    correction = ((int32_t)pid->kp_q10 * error +
                  i_term +
                  (int32_t)pid->kd_q10 * derivative) / 1024;

    if (correction > pid->max_output) {
        correction = pid->max_output;
    } else if (correction < -pid->max_output) {
        correction = -pid->max_output;
    }

    return (int16_t)correction;
}

void SpeedPID_reset(SpeedPID *pid)
{
    pid->integral   = 0;
    pid->last_error = 0;
}
