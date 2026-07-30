#ifndef LINE_FOLLOW_H_
#define LINE_FOLLOW_H_

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 5U

/* Tunable line-follow parameters.
 * base_speed is normally supplied by track_fsm.c, so main.c usually tunes
 * kp_q10/kd_q10/max_speed/lost_turn_speed more often than this field.
 */
typedef struct {
    uint16_t base_speed;      /* fallback base speed */
    uint16_t max_speed;       /* highest logical wheel speed */
    int16_t kp_q10;           /* bigger = turns harder toward the line */
    int16_t ki_q10;           /* normally 0 at this stage */
    int16_t kd_q10;           /* bigger = less shaking, too big = sluggish */
    uint16_t lost_turn_speed; /* spin speed when all sensors see white */
} LineFollowConfig;

/* Live line-follow status for debugging.
 * raw_sensor_bits is the direct 5-bit input after board mapping.
 * sensor_bits may reuse the last valid value when a noisy pattern appears.
 */
typedef struct {
    uint8_t raw_sensor_bits;
    uint8_t sensor_bits;
    int16_t position;       /* left negative, center 0, right positive */
    int16_t error;          /* same direction as position */
    int16_t filtered_error; /* low-pass filtered error used by PID */
    int16_t last_error;
    int32_t integral;
    bool line_seen;         /* false means all white/lost line */
    bool sensor_valid;      /* false means separated noisy pattern */
    int16_t left_speed;     /* logical left wheel command */
    int16_t right_speed;    /* logical right wheel command */
} LineFollowState;

void LineFollow_init(LineFollowState *state);
void LineFollow_update(LineFollowState *state,
                       const LineFollowConfig *config,
                       uint8_t sensor_bits,
                       uint16_t base_speed);

#endif
