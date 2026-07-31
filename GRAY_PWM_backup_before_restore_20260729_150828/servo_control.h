#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

/* 舵机控制配置。
 *
 * 这个结构体只描述“舵机应该怎么被限制和换算”，不负责小球 PID。
 * 小球 PID 后面只需要算出“水管应该往哪边倾、倾多少”，然后调用
 * ServoControl_setOffsetDegX10() 把偏移量交给本模块。
 *
 * 角度单位统一使用“度 * 10”的整数格式：
 *   900  表示 90.0 度
 *   -50  表示 -5.0 度偏移
 *
 * 这样不用 float，适合 MSPM0 这种没有硬件浮点的控制器，也方便在
 * CCS watch 窗口里直接看数值。
 */
typedef struct {
    int16_t  min_angle_x10;      /* 最小允许角度；限制水管最大反向倾角 */
    int16_t  center_angle_x10;   /* 机械中位角；水管静止调平时优先改这里 */
    int16_t  max_angle_x10;      /* 最大允许角度；限制水管最大正向倾角 */
    uint16_t min_pulse_us;       /* 最小角度对应脉宽，常见舵机一般约 1000us */
    uint16_t center_pulse_us;    /* 中位角对应脉宽，常见舵机一般约 1500us */
    uint16_t max_pulse_us;       /* 最大角度对应脉宽，常见舵机一般约 2000us */
    uint16_t max_step_us;        /* 单次刷新最大脉宽变化量；越小越柔，越大响应越快 */
    bool     invert;             /* 舵机方向反了就置 true，不用反着改视觉 PID */
} ServoControlConfig;

/* 舵机运行状态。
 *
 * target 是上层控制想要的目标值，current 是已经真正输出给舵机的值。
 * 两者故意分开，是为了做“斜坡限速”：
 *   PID 可以瞬间给出新目标；
 *   舵机输出则按 max_step_us 一点点追过去。
 *
 * 这样水管不会突然猛抬，小球也不容易被一下甩出目标区。
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

/* 设置绝对角度。例如 900 表示让舵机去 90.0 度。 */
void ServoControl_setAngleDegX10(ServoControl *servo, int16_t angle_x10);

/* 设置相对中位的偏移角。视觉位置 PID 的输出推荐接这个接口。 */
void ServoControl_setOffsetDegX10(ServoControl *servo, int16_t offset_x10);

/* 直接设置 PWM 高电平脉宽，常用于第一次调舵机中位。 */
void ServoControl_setPulseUs(ServoControl *servo, uint16_t pulse_us);

/* 每个控制周期调用一次，把 current 慢慢推向 target 并输出到底层 PWM。 */
void ServoControl_update(ServoControl *servo);

/* 调试读取接口，方便同步到全局变量或 OLED/蓝牙显示。 */
uint16_t ServoControl_getCurrentPulseUs(const ServoControl *servo);
int16_t ServoControl_getCurrentAngleDegX10(const ServoControl *servo);

#endif
