#include "speed_control.h"

/*
 * 单轮速度 PID。
 *
 * 这个控制器只在循迹给出的轮速命令附近做小幅修正。
 * 循迹算法负责决定小车往哪里走；速度闭环只负责让真实轮速
 * 尽量跟上目标编码器 tick。
 */

/* 初始化一个单轮 PID 控制器。
 * 增益使用 Q10 定点数：真实增益 = gain_q10 / 1024。
 */
void SpeedPID_init(SpeedPID *pid, int16_t kp_q10, int16_t ki_q10,
                   int16_t kd_q10, int16_t max_output)
{
    pid->kp_q10 = kp_q10;
    pid->ki_q10 = ki_q10;
    pid->kd_q10 = kd_q10;
    pid->max_output = max_output;
    pid->integral = 0;
    pid->last_error = 0;
}

/* 计算单个车轮的 PWM 修正量。
 *
 * target 和 actual 使用同一个单位：每个控制周期的编码器 tick 数。
 * 返回值会在 main.c 中叠加到原始开环 PWM 命令上。
 */
int16_t SpeedPID_compute(SpeedPID *pid, int16_t target, int16_t actual)
{
    /* 误差为正，表示当前轮子比目标慢，需要加 PWM。 */
    int16_t error = (int16_t) (target - actual);

    /* 误差变化量是 D 项输入，用来抑制速度突变。 */
    int16_t error_delta = (int16_t) (error - pid->last_error);
    int32_t output;

    pid->last_error = error;

    /* 积分项用于消除左右电机长期不一致。 */
    pid->integral += error;

    /* 积分抗饱和：轮子卡住时不能积累出很大的历史修正。 */
    if (pid->integral > SPEED_PID_INTEGRAL_LIMIT) {
        pid->integral = SPEED_PID_INTEGRAL_LIMIT;
    } else if (pid->integral < -SPEED_PID_INTEGRAL_LIMIT) {
        pid->integral = -SPEED_PID_INTEGRAL_LIMIT;
    }

    /* Q10 定点 PID 输出。实车上 Ki 要小，避免和循迹抢控制权。 */
    output = ((int32_t) pid->kp_q10 * error +
              (int32_t) pid->ki_q10 * pid->integral +
              (int32_t) pid->kd_q10 * error_delta) / 1024L;

    /* 速度环不能压过转向环，所以要限制最大修正量。 */
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }

    return (int16_t) output;
}

/* 停车、重新起步或切换模式时，清空 PID 历史状态。 */
void SpeedPID_reset(SpeedPID *pid)
{
    pid->integral = 0;
    pid->last_error = 0;
}
