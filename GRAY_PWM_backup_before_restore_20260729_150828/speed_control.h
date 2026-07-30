#ifndef SPEED_CONTROL_H_
#define SPEED_CONTROL_H_

#include <stdint.h>

/*
 * 单轮 Q10 定点速度 PID 控制器。
 *
 * 本工程里的含义：
 *   target = 每 10 ms 控制周期希望达到的编码器 tick
 *   actual = 同一个 10 ms 周期内实测的编码器 tick
 *   output = 有符号 PWM 修正量，叠加到循迹给出的轮速命令上
 *
 * 速度环应该保持温和。如果 max_output 太大，它会和循迹转向环打架，
 * 小车就会重新开始左右扭。
 */
typedef struct {
    int16_t kp_q10;      /* Q10 格式比例增益 */
    int16_t ki_q10;      /* Q10 格式积分增益 */
    int16_t kd_q10;      /* Q10 格式微分增益 */
    int32_t integral;    /* 速度误差积分 */
    int16_t last_error;  /* 上一周期误差，用于 D 项 */
    int16_t max_output;  /* 最大 PWM 修正量 */
} SpeedPID;

/* 速度环积分限幅，用于防止积分饱和。 */
#define SPEED_PID_INTEGRAL_LIMIT 1200L

void     SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                       int16_t kd_q10, int16_t max_output);
int16_t  SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual);
void     SpeedPID_reset(SpeedPID *pid);

/* 旧的辅助换算函数。当前闭环代码主要使用 main.c 里的
 * SPEED_TARGET_TICKS_PER_1000，这样调速度时不用来回翻文件。
 */
#define SPEED_TICKS_PER_1000  55

static inline int16_t Encoder_ticksToSpeed(int32_t ticks)
{
    return (int16_t)((ticks * 1000) / SPEED_TICKS_PER_1000);
}

#endif
