#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 5U

/* 循迹可调参数。
 *
 * 这里的速度和转向值都是“逻辑值”，还没有经过电机左右交换和方向修正。
 * 控制循环通常会传入动态 base_speed，但这个结构体把主要调车旋钮放在一起。
 */
typedef struct {
    uint16_t base_speed;      /* 默认直行速度 */
    uint16_t max_speed;       /* 逻辑轮速上限 */
    int16_t kp_q10;           /* 转向力度，越大转得越猛 */
    int16_t ki_q10;           /* 转向积分，灰度循迹一般保持为 0 */
    int16_t kd_q10;           /* 转向阻尼，越大越能压住过冲 */
    uint16_t lost_turn_speed; /* 五路全白丢线时的原地找线速度 */
} LineFollowConfig;

/* 循迹实时状态，用于调试、OLED 和蓝牙显示。
 *
 * raw_sensor_bits 是板级映射后的五路原始黑线位。
 * sensor_bits 是算法实际采用的图案；如果当前采样离散/噪声明显，
 * 它可能沿用上一拍有效图案。
 */
typedef struct {
    uint8_t raw_sensor_bits; /* 原始 L2/L1/C/R1/R2 黑线位 */
    uint8_t sensor_bits;     /* 本周期实际用于控制的图案 */
    int16_t position;        /* 解码后的位置：左负右正 */
    int16_t error;           /* 滤波后的最终转向误差 */
    int16_t filtered_error;  /* PID 使用的低通滤波位置 */
    int16_t last_error;      /* 上一周期误差，用于 D 项 */
    int32_t integral;        /* 误差积分，用于可选 Ki */
    bool line_seen;          /* false 表示五路全白/丢线 */
    bool sensor_valid;       /* false 表示当前图案离散、像噪声 */
    int16_t left_speed;      /* 左轮逻辑目标速度 */
    int16_t right_speed;     /* 右轮逻辑目标速度 */
} LineFollowState;

void LineFollow_init(LineFollowState *state);
void LineFollow_update(LineFollowState *state,
                       const LineFollowConfig *config,
                       uint8_t sensor_bits,
                       uint16_t base_speed);

#endif
