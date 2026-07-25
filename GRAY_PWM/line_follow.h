#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 5U

typedef struct {
    uint16_t base_speed;
    uint16_t max_speed;
    int16_t kp_q10;
    int16_t ki_q10;
    int16_t kd_q10;
    uint16_t lost_turn_speed;
} LineFollowConfig;

typedef struct {
    uint8_t raw_sensor_bits;
    uint8_t sensor_bits;
    int16_t position;
    int16_t error;
    int16_t last_error;
    int32_t integral;
    bool line_seen;
    bool sensor_valid;
    int16_t left_speed;
    int16_t right_speed;
} LineFollowState;

void LineFollow_init(LineFollowState *state);
void LineFollow_update(LineFollowState *state,
                       const LineFollowConfig *config,
                       uint8_t sensor_bits);

#endif
