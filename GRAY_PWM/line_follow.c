#include "line_follow.h"

/* Bit definition after board_port.c has corrected real sensor wiring.
 * A detected black line is represented as bit = 1.
 * Expected single-sensor values:
 *   L2=1, L1=2, C=4, R1=8, R2=16, all white=0, all black=31.
 */
#define BIT_SENSOR_L2 (1U << 0)
#define BIT_SENSOR_L1 (1U << 1)
#define BIT_SENSOR_C  (1U << 2)
#define BIT_SENSOR_R1 (1U << 3)
#define BIT_SENSOR_R2 (1U << 4)
#define SENSOR_MASK   0x1FU

/* Clamp wheel command to [0, max_speed].
 * Line-follow output is logical wheel speed, not the final motor sign.
 */
static uint16_t clamp_u16(int32_t value, uint16_t max_value)
{
    if (value < 0) {
        return 0;
    }
    if (value > (int32_t) max_value) {
        return max_value;
    }
    return (uint16_t) value;
}

/* Reject impossible separated patterns, for example L2+R2 without middle bits.
 * This helps ignore short noise caused by sensor shaking or bad height.
 */
static bool sensor_mask_is_contiguous(uint8_t sensor_bits)
{
    sensor_bits &= SENSOR_MASK;

    if (sensor_bits == 0U) {
        return true;
    }

    while ((sensor_bits & 1U) == 0U) {
        sensor_bits >>= 1;
    }
    while ((sensor_bits & 1U) != 0U) {
        sensor_bits >>= 1;
    }

    return sensor_bits == 0U;
}

/* Lost-line action: spin in the direction of the last known error.
 * If this does not happen during testing, first check g_run_enabled is true.
 */
static void set_lost_line_speed(LineFollowState *state,
                                const LineFollowConfig *config)
{
    if (state->last_error >= 0) {
        state->left_speed = (int16_t) config->lost_turn_speed;
        state->right_speed = -(int16_t) config->lost_turn_speed;
    } else {
        state->left_speed = -(int16_t) config->lost_turn_speed;
        state->right_speed = (int16_t) config->lost_turn_speed;
    }
}

void LineFollow_init(LineFollowState *state)
{
    state->raw_sensor_bits = 0;
    state->sensor_bits = 0;
    state->position = 0;
    state->error = 0;
    state->last_error = 0;
    state->integral = 0;
    state->line_seen = false;
    state->sensor_valid = false;
    state->left_speed = 0;
    state->right_speed = 0;
}

/* Main line-follow algorithm.
 * Input: 5-bit gray sensor value.
 * Output: logical left_speed/right_speed for the next control cycle.
 */
void LineFollow_update(LineFollowState *state,
                       const LineFollowConfig *config,
                       uint8_t sensor_bits,
                       uint16_t base_speed)
{
    /* Sensor position weights: left is negative, right is positive. */
    static const int16_t weights[LINE_SENSOR_COUNT] = {-2000, -1000, 0, 1000, 2000};
    int32_t weighted_sum = 0;
    uint8_t active_count = 0;
    uint8_t accepted_bits;

    sensor_bits &= SENSOR_MASK;
    state->raw_sensor_bits = sensor_bits;
    state->sensor_valid = sensor_mask_is_contiguous(sensor_bits);
    accepted_bits = sensor_bits;

    /* For noisy non-contiguous patterns, keep last valid line if we have one. */
    if (!state->sensor_valid) {
        accepted_bits = state->line_seen ? state->sensor_bits : 0U;
    }
    state->sensor_bits = accepted_bits;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if ((accepted_bits & (1U << i)) != 0U) {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    /* all white: no sensor sees black line, so enter lost-line spin. */
    if (active_count == 0U) {
        state->line_seen = false;
        set_lost_line_speed(state, config);
        return;
    }

    state->line_seen = true;
    state->position = (int16_t) (weighted_sum / active_count);
    state->error = state->position;

    /* D term is limited so a sudden jump from C to L2/R2 does not kick too hard. */
    int16_t derivative = state->error - state->last_error;
    if (derivative > 1000) {
        derivative = 1000;
    } else if (derivative < -1000) {
        derivative = -1000;
    }

    /* Integral clamp. ki_q10 is kept at 0 during basic tuning. */
    state->integral += state->error;
    {
        int32_t max_integral = ((int32_t) config->max_speed * 1024) / 2;
        if (state->integral > max_integral) {
            state->integral = max_integral;
        } else if (state->integral < -max_integral) {
            state->integral = -max_integral;
        }
    }

    /* Q10 PID correction: real gain = parameter / 1024. */
    int32_t correction = ((int32_t) config->kp_q10 * state->error +
                          (int32_t) config->ki_q10 * state->integral +
                          (int32_t) config->kd_q10 * derivative) / 1024;
    state->last_error = state->error;

    state->left_speed = (int16_t) clamp_u16((int32_t) base_speed + correction,
                                            config->max_speed);
    state->right_speed = (int16_t) clamp_u16((int32_t) base_speed - correction,
                                             config->max_speed);

    /* Outer sensors mean the line is far from center. Use a softer forced turn
     * instead of max/very-low speed, otherwise the car shakes hard.
     */
    uint16_t outer_slow_speed = base_speed / 2U;
    uint16_t outer_fast_speed = (uint16_t) ((base_speed + config->max_speed) / 2U);

    if ((accepted_bits == BIT_SENSOR_L2) || (accepted_bits == (BIT_SENSOR_L2 | BIT_SENSOR_L1))) {
        state->left_speed = outer_slow_speed;
        state->right_speed = outer_fast_speed;
    } else if ((accepted_bits == BIT_SENSOR_R2) || (accepted_bits == (BIT_SENSOR_R2 | BIT_SENSOR_R1))) {
        state->left_speed = outer_fast_speed;
        state->right_speed = outer_slow_speed;
    } else if (accepted_bits == (BIT_SENSOR_L2 | BIT_SENSOR_L1 | BIT_SENSOR_C |
                                 BIT_SENSOR_R1 | BIT_SENSOR_R2)) {
        state->left_speed = base_speed / 2;
        state->right_speed = base_speed / 2;
    }
}