#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/* 舵机控制配置。
 *
 * 角度使用“度 * 10”的整数格式，例如 900 表示 90.0 度。
 * 这样不用 float，适合 MSPM0 这种没有硬件浮点的控制器。
 */
typedef struct {
    int16_t  min_angle_x10;      /* 允许输出的最小舵机角度 */
    int16_t  center_angle_x10;   /* 机械中位角，调平水管时主要改这里 */
    int16_t  max_angle_x10;      /* 允许输出的最大舵机角度 */
    uint16_t min_pulse_us;       /* 最小角度对应的高电平脉宽，常见为 1000us */
    uint16_t center_pulse_us;    /* 中位角对应的高电平脉宽，常见为 1500us */
    uint16_t max_pulse_us;       /* 最大角度对应的高电平脉宽，常见为 2000us */
    uint16_t max_step_us;        /* 每个控制周期最多变化多少 us，用来保护舵机并减小冲击 */
    bool     invert;             /* 舵机方向反了就置 true，不用改 PID 符号 */
} ServoControlConfig;

/* 舵机运行状态。
 *
 * target 是 PID 或上层控制想要的目标，current 是已经输出到舵机的值。
 * current 会按 max_step_us 慢慢追 target，避免舵机角度突变。
 */
typedef struct {
    ServoControlConfig config;
    int16_t  target_angle_x10;   /* 目标角度，单位：度 * 10 */
    int16_t  current_angle_x10;  /* 当前输出角度，单位：度 * 10 */
    uint16_t target_pulse_us;    /* 目标脉宽，单位：us */
    uint16_t current_pulse_us;   /* 当前输出脉宽，单位：us */
} ServoControl;

void ServoControl_init(ServoControl *servo, const ServoControlConfig *config);
void ServoControl_center(ServoControl *servo);
void ServoControl_setAngleDegX10(ServoControl *servo, int16_t angle_x10);
void ServoControl_setOffsetDegX10(ServoControl *servo, int16_t offset_x10);
void ServoControl_setPulseUs(ServoControl *servo, uint16_t pulse_us);
void ServoControl_update(ServoControl *servo);
uint16_t ServoControl_getCurrentPulseUs(const ServoControl *servo);
int16_t ServoControl_getCurrentAngleDegX10(const ServoControl *servo);

#endif
