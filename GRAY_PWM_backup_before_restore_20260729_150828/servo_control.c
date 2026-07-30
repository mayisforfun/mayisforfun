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
 * 后面现场调平水管时，优先微调 center_angle_x10。
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

/* 保护配置参数，防止现场调参时把角度范围或脉宽范围写反。 */
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

    if (config->invert) {
        limited_angle = (int16_t) (config->center_angle_x10 -
            (limited_angle - config->center_angle_x10));
        limited_angle = clamp_i16(limited_angle,
                                  config->min_angle_x10,
                                  config->max_angle_x10);
    }

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

    if (angle_range > 0L) {
        pulse += (angle_delta * pulse_range) / angle_range;
    }

    return clamp_u16(pulse, config->min_pulse_us, config->max_pulse_us);
}

/* 脉宽反算角度，主要用于直接按脉宽调试时同步状态显示。 */
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

    Board_setServoPulseUs(servo->current_pulse_us);
}

/* 舵机回中。小球平衡题里，视觉丢球或停止控制时优先调用这个函数。 */
void ServoControl_center(ServoControl *servo)
{
    ServoControl_setAngleDegX10(servo, servo->config.center_angle_x10);
}

/* 设置绝对角度，单位是“度 * 10”。 */
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
 */
void ServoControl_setOffsetDegX10(ServoControl *servo, int16_t offset_x10)
{
    ServoControl_setAngleDegX10(servo,
        (int16_t) (servo->config.center_angle_x10 + offset_x10));
}

/* 直接设置舵机脉宽，适合第一次调舵机中位和方向时使用。 */
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
 */
void ServoControl_update(ServoControl *servo)
{
    if (servo->current_pulse_us < servo->target_pulse_us) {
        uint32_t next_pulse = (uint32_t) servo->current_pulse_us +
            servo->config.max_step_us;

        if (next_pulse > servo->target_pulse_us) {
            next_pulse = servo->target_pulse_us;
        }
        servo->current_pulse_us = (uint16_t) next_pulse;
    } else if (servo->current_pulse_us > servo->target_pulse_us) {
        uint16_t step = servo->config.max_step_us;

        if ((uint32_t) servo->current_pulse_us <=
            ((uint32_t) servo->target_pulse_us + step)) {
            servo->current_pulse_us = servo->target_pulse_us;
        } else {
            servo->current_pulse_us = (uint16_t)
                (servo->current_pulse_us - step);
        }
    }

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
