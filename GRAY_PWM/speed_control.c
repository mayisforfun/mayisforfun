#include "speed_control.h"

void SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                   int16_t kd_q10, int16_t max_output)
{
    /* Q10 means real gain = value / 1024.
     * Example: kp_q10 = 180 means Kp is about 0.176.
     */
    pid->kp_q10     = kp_q10;
    pid->ki_q10     = ki_q10;
    pid->kd_q10     = kd_q10;
    pid->max_output = max_output;
    pid->integral   = 0;
    pid->last_error = 0;
}

int16_t SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual)
{
    /* target: wanted encoder ticks in this control cycle.
     * actual: measured encoder ticks in this control cycle.
     * output: PWM correction added to the open-loop PWM command.
     */
    int16_t error = target - actual;
    int16_t derivative = error - pid->last_error;
    int32_t correction;
    int32_t i_term = 0;

    pid->last_error = error;

    if (pid->ki_q10 != 0) {
        /* Anti-windup: limit integral so the I term cannot hold a huge output
         * after the wheel was blocked or lifted for a while.
         */
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
        /* Ki is intentionally kept at 0 during early tuning. */
        pid->integral = 0;
    }

    correction = ((int32_t)pid->kp_q10 * error +
                  i_term +
                  (int32_t)pid->kd_q10 * derivative) / 1024;

    /* Keep the speed loop as a small correction, not the main driver. */
    if (correction > pid->max_output) {
        correction = pid->max_output;
    } else if (correction < -pid->max_output) {
        correction = -pid->max_output;
    }

    return (int16_t)correction;
}

void SpeedPID_reset(SpeedPID *pid)
{
    /* Call this when enabling the car or changing modes. */
    pid->integral   = 0;
    pid->last_error = 0;
}
