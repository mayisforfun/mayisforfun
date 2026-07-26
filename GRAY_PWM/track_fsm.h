#ifndef TRACK_FSM_H_
#define TRACK_FSM_H_

#include <stdint.h>
#include "line_follow.h"

typedef enum {
    TRACK_STRAIGHT,
    TRACK_CURVE,
    TRACK_CROSS,
    TRACK_LOST,
} TrackState;

typedef struct {
    TrackState state;
    uint16_t   base_speed;          /* current recommended base speed      */
    uint8_t    state_counter;       /* consecutive cycles in condition     */
    uint8_t    confirm_threshold;   /* cycles needed to confirm transition */
    uint16_t   straight_speed;      /* base speed for straights            */
    uint16_t   curve_speed;         /* base speed for curves               */
    uint16_t   cross_speed;         /* base speed for crossing             */
    uint16_t   lost_speed;          /* base speed while searching          */
} TrackFSM;

void TrackFSM_init(TrackFSM *fsm);
void TrackFSM_update(TrackFSM *fsm, const LineFollowState *line,
                     uint16_t *out_base_speed);

#endif
