#ifndef TI_BALL_CONTROL_H_
#define TI_BALL_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>
#include "servo_control.h"

typedef enum {
    TI_BALL_TASK_IDLE = 0,
    TI_BALL_TASK_STATIC_PLUS5_TO_MINUS5 = 1,
    TI_BALL_TASK_STRAIGHT_HOLD_ZERO = 2,
    TI_BALL_TASK_CIRCLE_HOLD_ZERO = 3,
    TI_BALL_TASK_CIRCLE_HOLD_CAPTURED = 4,
} TIBallTask;

typedef struct {
    float kp_deg_per_cm;
    float ki_deg_per_cm_s;
    float kd_deg_per_cm_s;
    float integral_limit_cm_s;
    float max_tilt_deg;
    float deadband_cm;
    float position_alpha;
    float velocity_alpha;
    float pixels_per_cm;
    int8_t position_sign;
    int8_t servo_sign;
    uint16_t vision_timeout_ticks; /* 当前 TI 主循环 1 tick = 10 ms */
} TIBallControlConfig;

typedef struct {
    TIBallControlConfig config;
    TIBallTask task;
    bool origin_valid;
    bool vision_valid;
    float origin_pixel;
    float raw_position_cm;
    float position_cm;
    float velocity_cm_s;
    float target_cm;
    float integral;
    float previous_position_cm;
    uint32_t last_vision_tick;
    uint32_t last_update_tick;
    uint32_t task1_stable_start_tick;
    uint8_t task1_phase;
    int16_t output_tilt_deg_x10;
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
