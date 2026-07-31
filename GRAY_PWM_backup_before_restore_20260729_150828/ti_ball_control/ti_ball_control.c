#include "ti_ball_control.h"

#define TI_CONTROL_TICK_SECONDS       0.01f
#define TI_TASK1_TARGET_CM            5.0f
#define TI_TASK1_TOLERANCE_CM         0.4f
#define TI_TASK1_STABLE_TICKS         100U

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

TIBallControlConfig TIBallControl_defaultConfig(void)
{
    TIBallControlConfig config;

    /* 安全起调值。实车先把 Ki 设为 0 调 Kp/Kd，再加入很小的 Ki。 */
    config.kp_deg_per_cm = 1.8f;
    config.ki_deg_per_cm_s = 0.08f;
    config.kd_deg_per_cm_s = 0.65f;
    config.integral_limit_cm_s = 20.0f;
    config.max_tilt_deg = 10.0f;
    config.deadband_cm = 0.15f;
    config.position_alpha = 0.35f;
    config.velocity_alpha = 0.25f;
    config.pixels_per_cm = 20.0f;
    config.position_sign = 1;
    config.servo_sign = 1;
    config.vision_timeout_ticks = 15U;
    return config;
}

void TIBallControl_init(TIBallControl *control,
                        const TIBallControlConfig *config)
{
    control->config = (config != 0) ? *config : TIBallControl_defaultConfig();
    control->task = TI_BALL_TASK_IDLE;
    control->origin_valid = false;
    control->vision_valid = false;
    control->origin_pixel = 0.0f;
    control->raw_position_cm = 0.0f;
    control->position_cm = 0.0f;
    control->velocity_cm_s = 0.0f;
    control->target_cm = 0.0f;
    control->integral = 0.0f;
    control->previous_position_cm = 0.0f;
    control->last_vision_tick = 0U;
    control->last_update_tick = 0U;
    control->task1_stable_start_tick = 0U;
    control->task1_phase = 0U;
    control->output_tilt_deg_x10 = 0;
}

void TIBallControl_requestOriginCapture(TIBallControl *control)
{
    control->origin_valid = false;
    control->vision_valid = false;
    control->integral = 0.0f;
    control->velocity_cm_s = 0.0f;
    control->output_tilt_deg_x10 = 0;
}

void TIBallControl_pushVision(TIBallControl *control,
                              uint16_t pixel_position,
                              uint32_t now_tick)
{
    float measured_cm;

    if (control->config.pixels_per_cm <= 0.0f) return;

    if (!control->origin_valid) {
        control->origin_pixel = (float) pixel_position;
        control->origin_valid = true;
        control->position_cm = 0.0f;
        control->previous_position_cm = 0.0f;
    }

    measured_cm = ((float) pixel_position - control->origin_pixel) /
                  control->config.pixels_per_cm;
    measured_cm *= (float) control->config.position_sign;
    control->raw_position_cm = measured_cm;

    if (!control->vision_valid) {
        control->position_cm = measured_cm;
        control->previous_position_cm = measured_cm;
    } else {
        control->position_cm += control->config.position_alpha *
                                (measured_cm - control->position_cm);
    }

    control->vision_valid = true;
    control->last_vision_tick = now_tick;
}

bool TIBallControl_setTask(TIBallControl *control,
                           TIBallTask task,
                           uint32_t now_tick)
{
    if ((task < TI_BALL_TASK_IDLE) ||
        (task > TI_BALL_TASK_CIRCLE_HOLD_CAPTURED)) {
        return false;
    }
    if ((task == TI_BALL_TASK_CIRCLE_HOLD_CAPTURED) &&
        !control->vision_valid) {
        return false;
    }

    control->task = task;
    control->integral = 0.0f;
    control->velocity_cm_s = 0.0f;
    control->previous_position_cm = control->position_cm;
    control->last_update_tick = now_tick;
    control->task1_phase = 0U;
    control->task1_stable_start_tick = 0U;

    switch (task) {
    case TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5:
        control->target_cm = TI_TASK1_TARGET_CM;
        break;
    case TI_BALL_TASK_STRAIGHT_HOLD_ZERO:
    case TI_BALL_TASK_CIRCLE_HOLD_ZERO:
        control->target_cm = 0.0f;
        break;
    case TI_BALL_TASK_CIRCLE_HOLD_CAPTURED:
        control->target_cm = control->position_cm;
        break;
    default:
        control->target_cm = 0.0f;
        break;
    }
    return true;
}

static void update_task1(TIBallControl *control, uint32_t now_tick)
{
    float abs_error = control->target_cm - control->position_cm;
    if (abs_error < 0.0f) abs_error = -abs_error;

    if ((control->task1_phase == 0U) &&
        (abs_error <= TI_TASK1_TOLERANCE_CM)) {
        if (control->task1_stable_start_tick == 0U) {
            control->task1_stable_start_tick = now_tick;
        } else if ((uint32_t) (now_tick - control->task1_stable_start_tick) >=
                   TI_TASK1_STABLE_TICKS) {
            control->task1_phase = 1U;
            control->target_cm = -TI_TASK1_TARGET_CM;
            control->integral = 0.0f;
            control->task1_stable_start_tick = 0U;
        }
    } else if (abs_error > TI_TASK1_TOLERANCE_CM) {
        control->task1_stable_start_tick = 0U;
    }
}

void TIBallControl_update(TIBallControl *control,
                          uint32_t now_tick,
                          ServoControl *servo)
{
    uint32_t elapsed_ticks;
    float dt_s;
    float measured_velocity;
    float error;
    float tilt_deg;

    if ((control->task == TI_BALL_TASK_IDLE) ||
        !control->origin_valid || !control->vision_valid ||
        ((uint32_t) (now_tick - control->last_vision_tick) >
         control->config.vision_timeout_ticks)) {
        control->integral = 0.0f;
        control->velocity_cm_s = 0.0f;
        control->output_tilt_deg_x10 = 0;
        ServoControl_center(servo);
        return;
    }

    elapsed_ticks = now_tick - control->last_update_tick;
    control->last_update_tick = now_tick;
    if (elapsed_ticks == 0U) elapsed_ticks = 1U;
    if (elapsed_ticks > 5U) elapsed_ticks = 5U;
    dt_s = (float) elapsed_ticks * TI_CONTROL_TICK_SECONDS;

    if (control->task == TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5) {
        update_task1(control, now_tick);
    }

    measured_velocity = (control->position_cm -
                         control->previous_position_cm) / dt_s;
    control->previous_position_cm = control->position_cm;
    control->velocity_cm_s += control->config.velocity_alpha *
                              (measured_velocity - control->velocity_cm_s);

    error = control->target_cm - control->position_cm;
    if ((error > -control->config.deadband_cm) &&
        (error < control->config.deadband_cm)) {
        error = 0.0f;
    }

    control->integral += error * dt_s;
    control->integral = clamp_f32(control->integral,
                                  -control->config.integral_limit_cm_s,
                                  control->config.integral_limit_cm_s);

    /* 微分对测量速度作用，任务一从 +5 切到 -5 时不会产生设定值冲击。 */
    tilt_deg = control->config.kp_deg_per_cm * error +
               control->config.ki_deg_per_cm_s * control->integral -
               control->config.kd_deg_per_cm_s * control->velocity_cm_s;
    tilt_deg = clamp_f32(tilt_deg,
                         -control->config.max_tilt_deg,
                         control->config.max_tilt_deg);
    tilt_deg *= (float) control->config.servo_sign;

    control->output_tilt_deg_x10 = (int16_t) (tilt_deg * 10.0f);
    ServoControl_setOffsetDegX10(servo, control->output_tilt_deg_x10);
}
