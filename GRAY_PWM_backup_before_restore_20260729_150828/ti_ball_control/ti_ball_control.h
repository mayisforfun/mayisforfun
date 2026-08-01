#ifndef TI_BALL_CONTROL_H_
#define TI_BALL_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include "servo_control.h"

typedef enum {
    TI_BALL_TASK_IDLE = 0,                    /* 不做位置闭环，舵机回水平中位 */
    TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5 = 1, /* 题1：静止，先到 +5cm，再到 -5cm */
    TI_BALL_TASK_STRAIGHT_HOLD_ZERO = 2,     /* 题2：小车直行，小球保持初始零点 */
    TI_BALL_TASK_CIRCLE_HOLD_ZERO = 3,       /* 题3：小车绕圈，小球保持初始零点 */
    TI_BALL_TASK_CIRCLE_HOLD_CAPTURED = 4,   /* 题4：记住任意点，小车绕圈时保持该点 */
} TIBallTask;

/* 视觉位置—速度耦合控制参数。
 *
 * 数据流：
 *   K230 center_x
 *       -> 减去 origin_pixel
 *       -> pixels_per_cm 换算成 cm
 *       -> 位置/速度低通滤波
 *       -> Kp*位置偏差 - Kd*小球速度
 *       -> 直接生成相对水平点的 SG90 微小脉宽偏移
 *
 * 不使用传统位置式/增量式 PID，也不使用积分项。超限后锁定进入回中模式，
 * 以水管物理中点为终点，并用“期望速度随剩余距离减小”的轨迹提前刹车。
 */
typedef struct {
    float kp_us_per_cm;              /* 正常控制：位置偏差每1cm产生多少us */
    float kd_us_per_cm_s;            /* 正常控制：速度反馈每1cm/s产生多少us */
    float return_kp_us_per_cm;       /* 超限回中：预测位置偏差增益 */
    float return_kv_us_per_cm_s;     /* 超限回中：期望/实际速度差增益 */
    float return_speed_per_cm_s;     /* 回中剩余1cm允许多少cm/s期望速度 */
    float max_return_speed_cm_s;     /* 回中期望速度上限，限制加速阶段 */
    float prediction_time_s;         /* 按当前速度向前预测多少秒的位置 */
    float soft_limit_cm;             /* 相对物理中点超过此距离进入回中 */
    float return_position_tolerance_cm; /* 回到中点的位置容差 */
    float return_velocity_tolerance_cm_s; /* 回到中点的速度容差 */
    float deadband_cm;               /* 正常目标误差死区，减少舵机抖动 */
    float position_alpha;            /* 位置低通系数：越大越灵敏 */
    float velocity_alpha;            /* 速度低通系数：越大越灵敏 */
    float pixels_per_cm;             /* 图像中移动1cm对应多少像素 */
    float pipe_middle_pixel;         /* 水管物理中点对应的center_x */
    uint16_t max_pulse_offset_us;    /* 相对水平脉宽的最大正/负偏移 */
    int8_t position_sign;            /* center_x坐标方向相反时设为-1 */
    int8_t servo_sign;               /* 舵机机械方向相反时设为-1 */
    uint16_t vision_timeout_ticks; /* 丢失视觉多久后回中；1 tick = 10ms */
} TIBallControlConfig;

/* 运行状态。调试时可重点看 position_cm、target_cm、velocity_cm_s 和
 * output_pulse_offset_us，它们能判断问题发生在视觉、状态反馈还是舵机层。
 */
typedef struct {
    TIBallControlConfig config;
    TIBallTask task;                   /* 当前题目/控制模式 */
    bool origin_valid;                 /* 是否已经记录0cm对应的像素位置 */
    bool vision_valid;                 /* 最近是否收到过有效小球坐标 */
    float origin_pixel;                /* 按KEY1后第一帧center_x，定义为0cm */
    float raw_position_cm;             /* 未低通滤波的位置 */
    float position_cm;                 /* PID实际使用的滤波后位置 */
    float velocity_cm_s;               /* 由相邻视觉帧估算的小球速度 */
    float target_cm;                   /* 当前目标：题1先+5，稳定后切到-5 */
    float previous_position_cm;        /* 上一视觉帧位置，用于计算速度 */
    float physical_position_cm;        /* 相对水管物理中点的位置 */
    float predicted_position_cm;       /* 速度前视后的预测物理位置 */
    float desired_velocity_cm_s;       /* 回中轨迹当前允许的期望速度 */
    uint32_t last_vision_tick;         /* 最近一帧到达时间，用于丢球超时 */
    uint32_t last_update_tick;         /* 上一次PID更新时间 */
    uint32_t task1_stable_start_tick;  /* 进入目标容差区的起始时刻 */
    uint8_t task1_phase;               /* 0=前往+5cm，1=前往-5cm */
    bool return_active;                /* 已超限；锁定以物理中点为终点 */
    bool return_settled;               /* 已同时满足中点位置和低速条件 */
    int16_t output_pulse_offset_us;    /* 相对水平点的SG90脉宽偏移 */
} TIBallControl;

TIBallControlConfig TIBallControl_defaultConfig(void);
void TIBallControl_init(TIBallControl *control,
                        const TIBallControlConfig *config);
void TIBallControl_requestOriginCapture(TIBallControl *control);
void TIBallControl_pushVision(TIBallControl *control,
                              uint16_t pixel_position,
                              uint32_t now_tick);
bool TIBallControl_setTask(TIBallControl *control,
                           TIBallTask task,
                           uint32_t now_tick);
void TIBallControl_update(TIBallControl *control,
                          uint32_t now_tick,
                          ServoControl *servo);

#endif
