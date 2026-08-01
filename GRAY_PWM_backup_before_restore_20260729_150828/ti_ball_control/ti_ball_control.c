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

    /* 安全起调值。实车先把 Ki 设为 0 调 Kp/Kd，再加入很小的 Ki。
     *
     * P 决定“偏得越远，水管倾得越多”；
     * I 用来补偿水管无法完全调平造成的恒定漂移；
     * D 根据小球运动速度反向刹车，避免冲过目标。
     */
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
    /* KEY1调用这里后，不会立刻把当前旧数据当原点。
     * origin_valid清零，等K230下一帧新center_x到达时再记录，避免使用旧帧。
     */
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
        /* 到达+5cm后必须连续稳定1秒才切换到-5cm。
         * 只短暂经过容差区不算完成，可防止惯性冲过目标时误切阶段。
         */
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
    float error;
    float tilt_deg;

    if ((control->task == TI_BALL_TASK_IDLE) ||
        !control->origin_valid || !control->vision_valid ||
        ((uint32_t) (now_tick - control->last_vision_tick) >
         control->config.vision_timeout_ticks)) {
        /* 以下任一情况都不允许继续使用旧PID输出：
         *   1. 当前没有启用小球任务；
         *   2. KEY1后的原点还没捕获；
         *   3. 从未收到视觉坐标；
         *   4. K230坐标已经超时。
         * 此时清积分并让水管回到机械水平脉宽。
         */
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

    /* 位置误差定义为“目标 - 实际”。
     * 如果实物运动方向相反，优先改position_sign或servo_sign，不改此公式。
     */
    error = control->target_cm - control->position_cm;
    if ((error > -control->config.deadband_cm) &&
        (error < control->config.deadband_cm)) {
        error = 0.0f;
    }

    /* I项按时间累加，并通过integral_limit_cm_s做抗饱和限幅。 */
    control->integral += error * dt_s;
    control->integral = clamp_f32(control->integral,
                                  -control->config.integral_limit_cm_s,
                                  control->config.integral_limit_cm_s);

    /* 完整控制律：倾角 = Kp*误差 + Ki*积分 - Kd*小球速度。
     * D项对测量速度作用，而不是对误差直接求导，因此题1从+5切换到-5时
     * 不会因目标值突变产生很大的“微分冲击”。
     */
    tilt_deg = control->config.kp_deg_per_cm * error +
               control->config.ki_deg_per_cm_s * control->integral -
               control->config.kd_deg_per_cm_s * control->velocity_cm_s;
    tilt_deg = clamp_f32(tilt_deg,
                         -control->config.max_tilt_deg,
                         control->config.max_tilt_deg);
    tilt_deg *= (float) control->config.servo_sign;

    control->output_tilt_deg_x10 = (int16_t) (tilt_deg * 10.0f);
    /* PID只给“相对水平点的角度偏移”。真正的1900us中位、限幅、缓动和
     * 定时器输出都由ServoControl负责。
     */
    ServoControl_setOffsetDegX10(servo, control->output_tilt_deg_x10);
}
