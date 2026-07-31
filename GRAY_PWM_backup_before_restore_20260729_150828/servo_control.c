#include "servo_control.h"
#include "board_port.h"

/* 默认舵机参数。
 *
 * 普通 180 度舵机一般是：
 *   0 度   -> 1000us
 *   90 度  -> 1500us
 *   180 度 -> 2000us
 *
 * 小球水管控制不建议真的打满 0~180 度，所以默认只允许 60~120 度。
 * 原因：
 *   1. 水管倾角太大，小球会加速过猛，PID 很难刹住；
 *   2. 舵机打到机械极限会发热、抖动，甚至卡死；
 *   3. 视觉有延迟，过大的可用角度会放大超调。
 *
 * 后面现场调平水管时，优先微调 center_angle_x10 或 center_pulse_us。
 * 不建议一开始就改 PID，否则会把“机械不平”和“控制参数不对”混在一起。
 */
#define SERVO_DEFAULT_MIN_ANGLE_X10      600
#define SERVO_DEFAULT_CENTER_ANGLE_X10   900
#define SERVO_DEFAULT_MAX_ANGLE_X10      1200
#define SERVO_DEFAULT_MIN_PULSE_US       1000U
#define SERVO_DEFAULT_CENTER_PULSE_US    1500U
#define SERVO_DEFAULT_MAX_PULSE_US       2000U
#define SERVO_DEFAULT_MAX_STEP_US        20U

static int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return (int16_t) value;
}

static uint16_t clamp_u16(int32_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < (int32_t) min_value) {
        return min_value;
    }
    if (value > (int32_t) max_value) {
        return max_value;
    }
    return (uint16_t) value;
}

static ServoControlConfig default_config(void)
{
    ServoControlConfig config;

    config.min_angle_x10 = SERVO_DEFAULT_MIN_ANGLE_X10;
    config.center_angle_x10 = SERVO_DEFAULT_CENTER_ANGLE_X10;
    config.max_angle_x10 = SERVO_DEFAULT_MAX_ANGLE_X10;
    config.min_pulse_us = SERVO_DEFAULT_MIN_PULSE_US;
    config.center_pulse_us = SERVO_DEFAULT_CENTER_PULSE_US;
    config.max_pulse_us = SERVO_DEFAULT_MAX_PULSE_US;
    config.max_step_us = SERVO_DEFAULT_MAX_STEP_US;
    config.invert = false;

    return config;
}

/* 保护配置参数，防止现场调参时把角度范围或脉宽范围写反。
 *
 * 比如不小心写成 min_angle > center_angle，后面的线性换算会出现除法范围
 * 为负或者范围为 0 的情况。这里先把配置拉回一个能工作的状态，避免舵机
 * 输出异常。
 */
static void normalize_config(ServoControlConfig *config)
{
    if (config->min_angle_x10 > config->center_angle_x10) {
        config->min_angle_x10 = config->center_angle_x10;
    }
    if (config->max_angle_x10 < config->center_angle_x10) {
        config->max_angle_x10 = config->center_angle_x10;
    }

    if (config->min_pulse_us > config->center_pulse_us) {
        config->min_pulse_us = config->center_pulse_us;
    }
    if (config->max_pulse_us < config->center_pulse_us) {
        config->max_pulse_us = config->center_pulse_us;
    }

    if (config->max_step_us == 0U) {
        config->max_step_us = SERVO_DEFAULT_MAX_STEP_US;
    }
}

/* 角度转脉宽。
 *
 * 以中心点分成左右两段换算，这样即使机械结构左右行程不完全对称，
 * 也可以分别调整 min/center/max 三个脉宽。
 *
 * 举例：
 *   min_angle_x10    = 600   -> 60.0 度
 *   center_angle_x10 = 900   -> 90.0 度
 *   max_angle_x10    = 1200  -> 120.0 度
 *   min_pulse_us     = 1000
 *   center_pulse_us  = 1500
 *   max_pulse_us     = 2000
 *
 * 那么 105.0 度会落在 center~max 这一段，换算到约 1750us。
 *
 * invert 的作用：
 *   如果视觉 PID 判断“小球偏左，需要右端抬高”，但实物却反着动，
 *   不要先改 PID 正负号，先把 invert 取反，机械方向就统一了。
 */
static uint16_t angle_to_pulse_us(const ServoControlConfig *config,
                                  int16_t angle_x10)
{
    int16_t limited_angle = clamp_i16(angle_x10,
                                      config->min_angle_x10,
                                      config->max_angle_x10);
    int32_t angle_delta;
    int32_t angle_range;
    int32_t pulse_range;
    int32_t pulse;

    /* 方向反转围绕中心角做镜像，而不是简单取负。
     * 这样中心角仍然保持不变，只交换左右倾斜方向。
     */
    if (config->invert) {
        limited_angle = (int16_t) (config->center_angle_x10 -
            (limited_angle - config->center_angle_x10));
        limited_angle = clamp_i16(limited_angle,
                                  config->min_angle_x10,
                                  config->max_angle_x10);
    }

    /* 大于中心角走右半段，小于中心角走左半段。
     * 分段处理可以兼容左右机械行程不完全一样的支架。
     */
    if (limited_angle >= config->center_angle_x10) {
        angle_delta = (int32_t) limited_angle - config->center_angle_x10;
        angle_range = (int32_t) config->max_angle_x10 - config->center_angle_x10;
        pulse_range = (int32_t) config->max_pulse_us - config->center_pulse_us;
        pulse = config->center_pulse_us;
    } else {
        angle_delta = (int32_t) config->center_angle_x10 - limited_angle;
        angle_range = (int32_t) config->center_angle_x10 - config->min_angle_x10;
        pulse_range = (int32_t) config->center_pulse_us - config->min_pulse_us;
        pulse = config->center_pulse_us;
        pulse_range = -pulse_range;
    }

    /* 线性插值：角度偏移 / 角度范围 = 脉宽偏移 / 脉宽范围。 */
    if (angle_range > 0L) {
        pulse += (angle_delta * pulse_range) / angle_range;
    }

    return clamp_u16(pulse, config->min_pulse_us, config->max_pulse_us);
}

/* 脉宽反算角度，主要用于直接按脉宽调试时同步状态显示。
 *
 * 第一次接舵机时，经常需要直接试 1450us、1500us、1550us 找水管水平点。
 * 这时用 ServoControl_setPulseUs() 设脉宽后，状态里的角度也要跟着更新，
 * 否则 CCS watch 里看到的角度和真实输出会对不上。
 */
static int16_t pulse_to_angle_x10(const ServoControlConfig *config,
                                  uint16_t pulse_us)
{
    uint16_t limited_pulse = clamp_u16(pulse_us,
                                       config->min_pulse_us,
                                       config->max_pulse_us);
    int32_t pulse_delta;
    int32_t pulse_range;
    int32_t angle_range;
    int32_t angle;

    if (limited_pulse >= config->center_pulse_us) {
        pulse_delta = (int32_t) limited_pulse - config->center_pulse_us;
        pulse_range = (int32_t) config->max_pulse_us - config->center_pulse_us;
        angle_range = (int32_t) config->max_angle_x10 - config->center_angle_x10;
        angle = config->center_angle_x10;
    } else {
        pulse_delta = (int32_t) config->center_pulse_us - limited_pulse;
        pulse_range = (int32_t) config->center_pulse_us - config->min_pulse_us;
        angle_range = (int32_t) config->center_angle_x10 - config->min_angle_x10;
        angle = config->center_angle_x10;
        angle_range = -angle_range;
    }

    if (pulse_range > 0L) {
        angle += (pulse_delta * angle_range) / pulse_range;
    }

    if (config->invert) {
        angle = (int32_t) config->center_angle_x10 -
            (angle - config->center_angle_x10);
    }

    return clamp_i16(angle, config->min_angle_x10, config->max_angle_x10);
}

void ServoControl_init(ServoControl *servo, const ServoControlConfig *config)
{
    /* config 允许传 0。这样临时测试时可以快速用默认参数启动舵机。
     * 正式比赛建议像 main.c 里那样传入 g_ball_servo_config，所有参数集中放。
     */
    if (config == 0) {
        servo->config = default_config();
    } else {
        servo->config = *config;
    }

    normalize_config(&servo->config);

    servo->target_angle_x10 = servo->config.center_angle_x10;
    servo->current_angle_x10 = servo->config.center_angle_x10;
    servo->target_pulse_us = servo->config.center_pulse_us;
    servo->current_pulse_us = servo->config.center_pulse_us;

    /* 初始化后立即输出中位脉宽，让舵机上电先回到安全位置。
     * 注意：如果 board_port 里的 SERVO_PWM_ENABLE 还是 0，这里只是空输出，
     * 不会真的驱动舵机；等 SysConfig 配好独立 50Hz PWM 后再打开即可。
     */
    Board_setServoPulseUs(servo->current_pulse_us);
}

/* 舵机回中。小球平衡题里，视觉丢球或停止控制时优先调用这个函数。 */
void ServoControl_center(ServoControl *servo)
{
    ServoControl_setAngleDegX10(servo, servo->config.center_angle_x10);
}

/* 设置绝对角度，单位是“度 * 10”。
 *
 * 这个接口适合手动调试：
 *   ServoControl_setAngleDegX10(&servo, 900);  -> 去 90.0 度
 *   ServoControl_setAngleDegX10(&servo, 950);  -> 去 95.0 度
 *
 * 传入角度即使超出 min/max，也会被自动限幅。
 */
void ServoControl_setAngleDegX10(ServoControl *servo, int16_t angle_x10)
{
    servo->target_angle_x10 = clamp_i16(angle_x10,
                                        servo->config.min_angle_x10,
                                        servo->config.max_angle_x10);
    servo->target_pulse_us = angle_to_pulse_us(&servo->config,
                                               servo->target_angle_x10);
}

/* 设置相对中位的角度偏移。
 *
 * 后面小球 PID 的输出建议直接接到这里：
 *   PID 输出为正 -> 舵机向一侧倾斜
 *   PID 输出为负 -> 舵机向另一侧倾斜
 *
 * 推荐的小球控制结构：
 *   ball_error = target_pos - ball_pos;
 *   pid_output_x10 = BallPID_update(ball_error);
 *   ServoControl_setOffsetDegX10(&ball_servo, pid_output_x10);
 *
 * 也就是说，PID 不需要知道舵机真实中位是多少，只输出“相对水平点的倾角”。
 * 这样机械调平和 PID 调参互不干扰。
 */
void ServoControl_setOffsetDegX10(ServoControl *servo, int16_t offset_x10)
{
    ServoControl_setAngleDegX10(servo,
        (int16_t) (servo->config.center_angle_x10 + offset_x10));
}

/* 直接设置舵机脉宽，适合第一次调舵机中位和方向时使用。
 *
 * 调试建议：
 *   1. 先让舵机输出 1500us；
 *   2. 看水管是否水平；
 *   3. 如果不水平，试 1450us 或 1550us；
 *   4. 找到水平脉宽后，把它写回 center_pulse_us。
 *
 * 注意：这个接口会绕过角度换算直接设脉宽，但仍然会做 min/max 脉宽限幅。
 */
void ServoControl_setPulseUs(ServoControl *servo, uint16_t pulse_us)
{
    servo->target_pulse_us = clamp_u16(pulse_us,
                                       servo->config.min_pulse_us,
                                       servo->config.max_pulse_us);
    servo->target_angle_x10 = pulse_to_angle_x10(&servo->config,
                                                 servo->target_pulse_us);
}

/* 每个控制周期调用一次。
 *
 * 它不会直接跳到目标脉宽，而是按 max_step_us 慢慢靠近。
 * 这能减少水管突然大角度动作，小球也不容易被一下子甩飞。
 *
 * 当前主循环是 10ms 调一次，所以：
 *   max_step_us = 20
 *   1500us -> 1700us 需要 10 个周期，也就是约 100ms。
 *
 * 如果觉得舵机太慢，可以增大 max_step_us；
 * 如果觉得舵机动作太猛、导致小球抖，可以减小 max_step_us。
 */
void ServoControl_update(ServoControl *servo)
{
    /* 当前脉宽小于目标脉宽：每次最多加 max_step_us。 */
    if (servo->current_pulse_us < servo->target_pulse_us) {
        uint32_t next_pulse = (uint32_t) servo->current_pulse_us +
            servo->config.max_step_us;

        if (next_pulse > servo->target_pulse_us) {
            next_pulse = servo->target_pulse_us;
        }
        servo->current_pulse_us = (uint16_t) next_pulse;
    } else if (servo->current_pulse_us > servo->target_pulse_us) {
        /* 当前脉宽大于目标脉宽：每次最多减 max_step_us。 */
        uint16_t step = servo->config.max_step_us;

        if ((uint32_t) servo->current_pulse_us <=
            ((uint32_t) servo->target_pulse_us + step)) {
            servo->current_pulse_us = servo->target_pulse_us;
        } else {
            servo->current_pulse_us = (uint16_t)
                (servo->current_pulse_us - step);
        }
    }

    /* 同步当前角度，方便调试显示；最后把当前脉宽真正写到底层 PWM。 */
    servo->current_angle_x10 = pulse_to_angle_x10(&servo->config,
                                                  servo->current_pulse_us);
    Board_setServoPulseUs(servo->current_pulse_us);
}

uint16_t ServoControl_getCurrentPulseUs(const ServoControl *servo)
{
    return servo->current_pulse_us;
}

int16_t ServoControl_getCurrentAngleDegX10(const ServoControl *servo)
{
    return servo->current_angle_x10;
}
