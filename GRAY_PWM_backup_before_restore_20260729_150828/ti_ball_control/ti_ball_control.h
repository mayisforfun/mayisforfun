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

/* 视觉位置闭环的参数。
 *
 * 数据流：
 *   K230 center_x
 *       -> 减去 origin_pixel
 *       -> pixels_per_cm 换算成 cm
 *       -> 位置/速度低通滤波
 *       -> 位置 PID 算出水管倾角
 *       -> ServoControl 换算成 SG90 PWM 脉宽
 *
 * 这里的 PID 输出单位是“水管倾角（度）”，不是电机 PWM，也不是舵机脉宽。
 */
typedef struct {
    float kp_deg_per_cm;       /* P：每偏差 1cm，增加多少度倾角 */
    float ki_deg_per_cm_s;     /* I：消除水管轻微不平造成的长期静差 */
    float kd_deg_per_cm_s;     /* D：按小球速度刹车，抑制越过目标和来回振荡 */
    float integral_limit_cm_s; /* 积分限幅，防止长时间偏差导致积分饱和 */
    float max_tilt_deg;        /* PID允许的最大正/负倾角，保护机械结构 */
    float deadband_cm;         /* 误差小于该值时按0处理，减少舵机抖动 */
    float position_alpha;      /* 位置低通系数：越大越灵敏，越小越平滑 */
    float velocity_alpha;      /* 速度低通系数：越大越灵敏，越小越平滑 */
    float pixels_per_cm;       /* 标定比例：图像中移动1cm对应多少像素 */
    int8_t position_sign;      /* center_x增大方向与水管正方向相反时设为-1 */
    int8_t servo_sign;         /* PID正倾角的机械方向不对时设为-1 */
    uint16_t vision_timeout_ticks; /* 丢失视觉多久后回中；1 tick = 10ms */
} TIBallControlConfig;

/* 运行状态。调试时可重点看 position_cm、target_cm、velocity_cm_s 和
 * output_tilt_deg_x10，它们能判断问题发生在视觉、PID还是舵机输出层。
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
    float integral;                    /* I项累计误差 */
    float previous_position_cm;        /* 上一视觉帧位置，用于计算速度 */
    uint32_t last_vision_tick;         /* 最近一帧到达时间，用于丢球超时 */
    uint32_t last_update_tick;         /* 上一次PID更新时间 */
    uint32_t task1_stable_start_tick;  /* 进入目标容差区的起始时刻 */
    uint8_t task1_phase;               /* 0=前往+5cm，1=前往-5cm */
    int16_t output_tilt_deg_x10;       /* PID输出，单位0.1度；例如25=2.5度 */
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
