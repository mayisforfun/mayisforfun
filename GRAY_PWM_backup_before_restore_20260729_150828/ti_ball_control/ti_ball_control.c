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

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t round_f32_to_i16(float value)
{
    return (int16_t) ((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

TIBallControlConfig TIBallControl_defaultConfig(void)
{
    TIBallControlConfig config;

    /* 轻微扰动结构的保守起调值，输出单位直接是SG90脉宽us。
     * 正常状态使用：output = Kp*位置偏差 - Kd*小球坐标速度。
     * 超限状态使用预测位置与期望速度，使回中过程先加速、接近中点再减速。
     */
    config.kp_us_per_cm = 1.20f;
    config.kd_us_per_cm_s = 0.35f;
    config.return_kp_us_per_cm = 1.60f;
    config.return_kv_us_per_cm_s = 0.45f;
    config.return_speed_per_cm_s = 1.50f;
    config.max_return_speed_cm_s = 12.0f;
    config.prediction_time_s = 0.20f;
    config.soft_limit_cm = 10.0f;
    config.return_position_tolerance_cm = 0.40f;
    config.return_velocity_tolerance_cm_s = 1.0f;
    config.deadband_cm = 0.10f;
    config.position_alpha = 0.35f;
    config.velocity_alpha = 0.25f;
    config.pixels_per_cm = 20.0f;
    config.pipe_middle_pixel = 960.0f;
    config.max_pulse_offset_us = 20U;
    config.position_sign = 1;
    config.servo_sign = 1;
    config.vision_timeout_ticks = 50U;
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
    control->previous_position_cm = 0.0f;
    control->physical_position_cm = 0.0f;
    control->predicted_position_cm = 0.0f;
    control->desired_velocity_cm_s = 0.0f;
    control->last_vision_tick = 0U;
    control->last_update_tick = 0U;
    control->task1_stable_start_tick = 0U;
    control->task1_phase = 0U;
    control->return_active = false;
    control->return_settled = false;
    control->output_pulse_offset_us = 0;
}

void TIBallControl_requestOriginCapture(TIBallControl *control)
{
    /* KEY1调用这里后，不会立刻把当前旧数据当原点。
     * origin_valid清零，等K230下一帧新center_x到达时再记录，避免使用旧帧。
     */
    control->origin_valid = false;
    control->vision_valid = false;
    control->velocity_cm_s = 0.0f;
    control->return_active = false;
    control->return_settled = false;
    control->desired_velocity_cm_s = 0.0f;
    control->output_pulse_offset_us = 0;
}

void TIBallControl_pushVision(TIBallControl *control,
                              uint16_t pixel_position,
                              uint32_t now_tick)
{
    bool had_vision = control->vision_valid;
    uint32_t vision_elapsed_ticks = now_tick - control->last_vision_tick;
    float measured_cm;

    if (control->config.pixels_per_cm <= 0.0f) return;

    if (!control->origin_valid) {
        /* “初始位置为原点0”的实现位置：
         * KEY1之后收到的第一帧，无论小球在画面中央还是侧边，都定义为0cm。
         */
        control->origin_pixel = (float) pixel_position;
        control->origin_valid = true;
        control->position_cm = 0.0f;
        control->previous_position_cm = 0.0f;
    }

    measured_cm = ((float) pixel_position - control->origin_pixel) /
                  control->config.pixels_per_cm;
    measured_cm *= (float) control->config.position_sign;
    control->raw_position_cm = measured_cm;

    if (!had_vision) {
        control->position_cm = measured_cm;
        control->previous_position_cm = measured_cm;
        control->velocity_cm_s = 0.0f;
    } else {
        control->position_cm += control->config.position_alpha *
                                (measured_cm - control->position_cm);

        /* D项必须按真实视觉帧间隔计算速度。
         * 不能固定使用TI主循环的10ms：K230通常达不到100FPS，固定10ms会把
         * 速度算得过大，造成微分项突然猛推舵机。
         */
        if (vision_elapsed_ticks > 0U) {
            float vision_dt_s =
                (float) vision_elapsed_ticks * TI_CONTROL_TICK_SECONDS;
            float measured_velocity =
                (control->position_cm - control->previous_position_cm) /
                vision_dt_s;
            control->velocity_cm_s += control->config.velocity_alpha *
                (measured_velocity - control->velocity_cm_s);
        }
        control->previous_position_cm = control->position_cm;
    }

    /* 物理中点坐标独立于KEY1任务原点，并沿用同一个位置低通结果：
     * task position用于+5/-5等题目；physical position只用于安全超限回中。
     */
    control->physical_position_cm = control->position_cm +
        (control->origin_pixel - control->config.pipe_middle_pixel) /
        control->config.pixels_per_cm *
        (float) control->config.position_sign;

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
    control->velocity_cm_s = 0.0f;
    control->previous_position_cm = control->position_cm;
    control->last_update_tick = now_tick;
    control->task1_phase = 0U;
    control->task1_stable_start_tick = 0U;
    control->return_active = false;
    control->return_settled = false;
    control->desired_velocity_cm_s = 0.0f;
    control->output_pulse_offset_us = 0;

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
        /* 到达+5cm后必须连续稳定1秒才切换到-5cm。
         * 只短暂经过容差区不算完成，可防止惯性冲过目标时误切阶段。
         */
        if (control->task1_stable_start_tick == 0U) {
            control->task1_stable_start_tick = now_tick;
        } else if ((uint32_t) (now_tick - control->task1_stable_start_tick) >=
                   TI_TASK1_STABLE_TICKS) {
            control->task1_phase = 1U;
            control->target_cm = -TI_TASK1_TARGET_CM;
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
    float error;
    float pulse_offset_us;
    int16_t limited_offset_us;
    int32_t target_pulse_us;

    if ((control->task == TI_BALL_TASK_IDLE) ||
        !control->origin_valid || !control->vision_valid ||
        ((uint32_t) (now_tick - control->last_vision_tick) >
         control->config.vision_timeout_ticks)) {
        /* 以下任一情况都不允许继续使用旧PID输出：
         *   1. 当前没有启用小球任务；
         *   2. KEY1后的原点还没捕获；
         *   3. 从未收到视觉坐标；
         *   4. K230坐标已经超时。
         * 此时不能继续沿用旧速度或旧输出，让水管回到机械水平脉宽。
         */
        control->velocity_cm_s = 0.0f;
        control->desired_velocity_cm_s = 0.0f;
        control->output_pulse_offset_us = 0;
        ServoControl_center(servo);
        return;
    }

    control->last_update_tick = now_tick;

    /* 用当前速度预测prediction_time_s之后的位置。如果球正在快速向外运动，
     * 即使当前位置尚未越界，预测位置也会先越界，从而提前触发回拉。
     */
    control->predicted_position_cm = control->physical_position_cm +
        control->velocity_cm_s * control->config.prediction_time_s;

    if (!control->return_active &&
        ((abs_f32(control->physical_position_cm) >=
          control->config.soft_limit_cm) ||
         (abs_f32(control->predicted_position_cm) >=
          control->config.soft_limit_cm))) {
        /* 一旦超限便锁定回中，不在回中途中恢复原题目目标，避免再次加速冲出。
         * 重新按题目按键/重新setTask才会解除这个安全状态。
         */
        control->return_active = true;
        control->return_settled = false;
        control->task1_stable_start_tick = 0U;
    }

    if (control->return_active) {
        float predicted_error = -control->predicted_position_cm;

        /* 物理中点定义为0cm。期望速度与剩余距离耦合：
         *   离中点远 -> 期望速度大，先加速；
         *   接近中点 -> 期望速度自动变小，速度反馈促使水管提前刹车。
         */
        control->desired_velocity_cm_s = clamp_f32(
            -control->config.return_speed_per_cm_s *
                control->physical_position_cm,
            -control->config.max_return_speed_cm_s,
            control->config.max_return_speed_cm_s);

        pulse_offset_us =
            control->config.return_kp_us_per_cm * predicted_error +
            control->config.return_kv_us_per_cm_s *
                (control->desired_velocity_cm_s -
                 control->velocity_cm_s);

        /* 不能只用“经过中点”判断回中完成，还必须确认速度已经足够低。 */
        control->return_settled =
            (abs_f32(control->physical_position_cm) <=
             control->config.return_position_tolerance_cm) &&
            (abs_f32(control->velocity_cm_s) <=
             control->config.return_velocity_tolerance_cm_s);
    } else {
        if (control->task == TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5) {
            update_task1(control, now_tick);
        }

        /* 正常状态使用纯位置—速度反馈，不使用积分：
         *   output = Kp * (目标位置-实际位置) - Kd * 小球坐标速度。
         * 因为velocity是d(position)/dt，所以这里必须是减号；如果使用的是
         * d(error)/dt，数学上才写成加号。
         */
        error = control->target_cm - control->position_cm;
        if (abs_f32(error) < control->config.deadband_cm) {
            error = 0.0f;
        }
        control->desired_velocity_cm_s = 0.0f;
        pulse_offset_us = control->config.kp_us_per_cm * error -
            control->config.kd_us_per_cm_s * control->velocity_cm_s;
    }

    /* 轻微扰动结构不再换算大角度，直接输出水平点附近的微小脉宽偏移。 */
    pulse_offset_us = clamp_f32(
        pulse_offset_us,
        -(float) control->config.max_pulse_offset_us,
        (float) control->config.max_pulse_offset_us);
    pulse_offset_us *= (float) control->config.servo_sign;
    limited_offset_us = round_f32_to_i16(pulse_offset_us);
    control->output_pulse_offset_us = limited_offset_us;

    target_pulse_us = (int32_t) servo->config.center_pulse_us +
                      (int32_t) limited_offset_us;
    if (target_pulse_us < 0L) target_pulse_us = 0L;
    ServoControl_setPulseUs(servo, (uint16_t) target_pulse_us);
}
