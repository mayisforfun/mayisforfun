#include "line_follow.h"

/*
 * 五路灰度循迹算法。
 *
 * 输入约定：
 *   board_port.c 已经把真实 GPIO 电平转换成逻辑黑线位。
 *   bit = 1 表示对应灰度传感器检测到黑线。
 *
 * 控制思路：
 *   1. 把 5 位灰度状态解码成带正负的线位置。
 *      负数表示黑线在小车左侧，正数表示黑线在小车右侧。
 *   2. 对线位置做低通滤波，避免被单次抖动带偏。
 *   3. 用滤波后的误差做转向 PID。
 *   4. 黑线偏得越远，基础速度自动降得越多。
 *   5. 把基础速度和转向修正量混合成左右轮目标速度。
 */

/* 逻辑传感器位：L2 L1 C R1 R2。 */
#define BIT_SENSOR_L2 (1U << 0)
#define BIT_SENSOR_L1 (1U << 1)
#define BIT_SENSOR_C  (1U << 2)
#define BIT_SENSOR_R1 (1U << 3)
#define BIT_SENSOR_R2 (1U << 4)
#define SENSOR_MASK   0x1FU

/* 给 PID 使用的虚拟位置坐标，数值越大表示偏得越远。 */
#define POSITION_L2       (-1800)
#define POSITION_L1       (-800)
#define POSITION_CENTER   (0)
#define POSITION_R1       (800)
#define POSITION_R2       (1800)

/* 中心死区：当 C 看到黑线且位置接近 0 时，清掉转向记忆。
 * 这样小车回到中间后不会继续在 L1 和 R1 之间来回摆头。
 */
#define CENTER_DEADBAND        120

/* 一阶低通滤波：
 * filtered = (old * FILTER_OLD_WEIGHT + new) / FILTER_TOTAL_WEIGHT。
 * 旧值权重越大，直线越稳，但过弯反应会更慢。
 */
#define FILTER_OLD_WEIGHT      2
#define FILTER_TOTAL_WEIGHT    3

/* 循迹 PID 的积分限幅。Ki 通常保持 0；如果后面开启小 Ki，
 * 这里可以防止积分越积越大。
 */
#define INTEGRAL_LIMIT         6000L

/* 有符号限幅，用于限制 PID 修正量和误差范围。 */
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

/* 无符号限幅，用于限制前进方向的轮速命令。 */
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

/* 统计当前五个灰度传感器里有几个检测到黑线。 */
static uint8_t count_bits5(uint8_t bits)
{
    uint8_t count = 0;

    bits &= SENSOR_MASK;
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if ((bits & (1U << i)) != 0U) {
            count++;
        }
    }

    return count;
}

/* 剔除明显不合理的离散图案，比如只有 L2+R2 同时为黑而中间全白。
 * 正常黑线图案应该是连续的；离散图案通常来自震动、高度不对、
 * 反光或者接线噪声。
 */
static bool bits_are_contiguous(uint8_t bits)
{
    bits &= SENSOR_MASK;

    if (bits == 0U) {
        return true;
    }

    while ((bits & 1U) == 0U) {
        bits >>= 1;
    }
    while ((bits & 1U) != 0U) {
        bits >>= 1;
    }

    return bits == 0U;
}

/* 通用兜底位置计算。对所有检测到黑线的传感器做加权平均，
 * 这样较宽的黑线图案也能得到一个合理中心。
 */
static int16_t weighted_position(uint8_t bits)
{
    static const int16_t weights[LINE_SENSOR_COUNT] = {
        POSITION_L2, POSITION_L1, POSITION_CENTER, POSITION_R1, POSITION_R2
    };
    int32_t sum = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++) {
        if ((bits & (1U << i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    if (count == 0U) {
        return 0;
    }

    return (int16_t) (sum / count);
}

/* 把灰度图案转换成线位置。
 *
 * 常见图案单独列出来处理，保证车的动作可预期：
 *   只有 C             -> 居中
 *   L1+C / C+R1       -> 小偏差
 *   L1 / R1           -> 中等偏差
 *   L2 / R2           -> 大偏差，需要明显转向
 * 未列出的连续图案使用加权平均兜底。
 */
static int16_t decode_line_position(uint8_t bits)
{
    bits &= SENSOR_MASK;

    switch (bits) {
    case BIT_SENSOR_C:
    case (BIT_SENSOR_L1 | BIT_SENSOR_C | BIT_SENSOR_R1):
    case SENSOR_MASK:
        return 0;

    case (BIT_SENSOR_L1 | BIT_SENSOR_C):
        return -350;
    case (BIT_SENSOR_C | BIT_SENSOR_R1):
        return 350;

    case BIT_SENSOR_L1:
        return POSITION_L1;
    case BIT_SENSOR_R1:
        return POSITION_R1;

    case (BIT_SENSOR_L2 | BIT_SENSOR_L1):
        return -1300;
    case (BIT_SENSOR_R1 | BIT_SENSOR_R2):
        return 1300;

    case BIT_SENSOR_L2:
        return POSITION_L2;
    case BIT_SENSOR_R2:
        return POSITION_R2;

    case (BIT_SENSOR_L2 | BIT_SENSOR_L1 | BIT_SENSOR_C):
        return -900;
    case (BIT_SENSOR_C | BIT_SENSOR_R1 | BIT_SENSOR_R2):
        return 900;

    default:
        return weighted_position(bits);
    }
}

/* 动态速度规划。
 * 黑线离中心越远，基础速度越低。这样过弯时不会车速太快，
 * 导致转向还没跟上就冲出线。
 */
static uint16_t speed_for_error(uint16_t base_speed, int16_t error)
{
    int16_t abs_error = (error >= 0) ? error : (int16_t) -error;

    if (abs_error >= 1400) {
        return (uint16_t) (((uint32_t) base_speed * 65U) / 100U);
    }
    if (abs_error >= 800) {
        return (uint16_t) (((uint32_t) base_speed * 78U) / 100U);
    }
    if (abs_error >= 350) {
        return (uint16_t) (((uint32_t) base_speed * 88U) / 100U);
    }
    return base_speed;
}

/* 丢线恢复。
 * 如果五路全白，就朝上一次看到黑线的方向原地找线。
 * 这里故意写得简单确定，方便电赛现场排查。
 */
static void set_lost_line_speed(LineFollowState *state,
                                const LineFollowConfig *config)
{
    if (state->last_error < 0) {
        state->left_speed = -(int16_t) config->lost_turn_speed;
        state->right_speed = (int16_t) config->lost_turn_speed;
    } else {
        state->left_speed = (int16_t) config->lost_turn_speed;
        state->right_speed = -(int16_t) config->lost_turn_speed;
    }
}

/* 新一轮运行前清空所有循迹状态。 */
void LineFollow_init(LineFollowState *state)
{
    state->raw_sensor_bits = 0;
    state->sensor_bits = 0;
    state->position = 0;
    state->error = 0;
    state->filtered_error = 0;
    state->last_error = 0;
    state->integral = 0;
    state->line_seen = false;
    state->sensor_valid = false;
    state->left_speed = 0;
    state->right_speed = 0;
}

/* 循迹主更新函数，每个控制周期调用一次。
 *
 * 输出：
 *   state->left_speed 和 state->right_speed 是逻辑轮速命令。
 *   后面的板级电机层会再处理左右电机交换和方向符号。
 */
void LineFollow_update(LineFollowState *state,
                       const LineFollowConfig *config,
                       uint8_t sensor_bits,
                       uint16_t base_speed)
{
    uint8_t accepted_bits;
    int16_t position;
    int16_t derivative;
    int32_t correction;
    int32_t max_correction;
    uint16_t drive_speed;
    bool center_seen;

    /* 只保留五路有效位，同时保存原始值，方便 OLED/蓝牙调试。 */
    sensor_bits &= SENSOR_MASK;
    state->raw_sensor_bits = sensor_bits;
    state->sensor_valid = bits_are_contiguous(sensor_bits);

    /* 如果某一拍读到了物理上不合理的图案，就沿用上一次有效图案，
     * 避免一个噪声点把车头猛地拉偏。
     */
    if (!state->sensor_valid && state->line_seen) {
        accepted_bits = state->sensor_bits;
    } else {
        accepted_bits = sensor_bits;
    }
    state->sensor_bits = accepted_bits;

    /* 五路全白表示丢线。没有线时不跑正常 PID，改用找线动作。 */
    if (count_bits5(accepted_bits) == 0U) {
        state->line_seen = false;
        state->integral = 0;
        set_lost_line_speed(state, config);
        return;
    }

    state->line_seen = true;
    center_seen = (accepted_bits & BIT_SENSOR_C) != 0U;
    position = decode_line_position(accepted_bits);
    state->position = position;

    /* 中间传感器优先：当 C 看到黑线且位置接近 0 时，立刻清掉转向记忆。
     * 这是专门压住“回正后还在左右摆”的关键处理。
     */
    if (center_seen && (position > -CENTER_DEADBAND) && (position < CENTER_DEADBAND)) {
        state->filtered_error = 0;
        state->integral = 0;
    } else {
        /* 平滑误差，这是抑制摆头的主要滤波。 */
        state->filtered_error = (int16_t)
            ((((int32_t) state->filtered_error * FILTER_OLD_WEIGHT) + position) /
             FILTER_TOTAL_WEIGHT);
    }

    /* PID 三项：error 是线偏移量，derivative 是偏移变化速度。 */
    state->error = state->filtered_error;
    derivative = (int16_t) (state->error - state->last_error);
    state->last_error = state->error;

    /* 只有看到有效线后才积分；限幅用于防止开启 Ki 后积分饱和。 */
    state->integral += state->error;
    if (state->integral > INTEGRAL_LIMIT) {
        state->integral = INTEGRAL_LIMIT;
    } else if (state->integral < -INTEGRAL_LIMIT) {
        state->integral = -INTEGRAL_LIMIT;
    }

    /* 根据当前转向难度选择前进速度。 */
    drive_speed = speed_for_error(base_speed, state->error);
    if (drive_speed > config->max_speed) {
        drive_speed = config->max_speed;
    }

    /* Q10 定点 PID：真实增益 = gain_q10 / 1024。 */
    correction = ((int32_t) config->kp_q10 * state->error +
                  (int32_t) config->ki_q10 * state->integral +
                  (int32_t) config->kd_q10 * derivative) / 1024L;

    /* 转向修正量最多为当前前进速度的 80%。
     * 这样可以避免某一侧轮子被修得过猛，重新引入摆头。
     */
    max_correction = ((int32_t) drive_speed * 8L) / 10L;
    correction = clamp_i16(correction, (int16_t) -max_correction,
                           (int16_t) max_correction);

    /* 差速混控：
     *   correction > 0 表示黑线在右边，左轮加速、右轮减速，小车右转。
     */
    state->left_speed = (int16_t) clamp_u16((int32_t) drive_speed + correction,
                                            config->max_speed);
    state->right_speed = (int16_t) clamp_u16((int32_t) drive_speed - correction,
                                             config->max_speed);
}
