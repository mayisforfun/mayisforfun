#include "track_fsm.h"

#define ERROR_CURVE_ENTER  700
#define ERROR_CURVE_EXIT   300
#define CURVE_ENTER_CYCLES 3
#define CURVE_EXIT_CYCLES  6
#define LOST_ENTER_CYCLES  8
#define LOST_EXIT_CYCLES   3
#define CROSS_ENTER_CYCLES 2
#define CROSS_TIMEOUT      25

void TrackFSM_init(TrackFSM *fsm)
{
    fsm->state             = TRACK_STRAIGHT;
    fsm->base_speed        = 310;
    fsm->state_counter     = 0;
    fsm->confirm_threshold = CURVE_ENTER_CYCLES;
    fsm->straight_speed    = 310;
    fsm->curve_speed       = 230;
    fsm->cross_speed       = 210;
    fsm->lost_speed        = 170;
}

static int16_t abs_i16(int16_t v)
{
    return (v >= 0) ? v : (int16_t) -v;
}

void TrackFSM_update(TrackFSM *fsm, const LineFollowState *line,
                     uint16_t *out_base_speed)
{
    bool all_black = (line->raw_sensor_bits == 0x1FU);
    bool line_lost = !line->line_seen;
    int16_t abs_err = abs_i16(line->error);

    switch (fsm->state) {

    case TRACK_STRAIGHT:
        if (line_lost) {
            fsm->state_counter++;
            if (fsm->state_counter >= LOST_ENTER_CYCLES) {
                fsm->state = TRACK_LOST;
                fsm->state_counter = 0;
                fsm->confirm_threshold = LOST_EXIT_CYCLES;
            }
        } else if (all_black) {
            fsm->state_counter++;
            if (fsm->state_counter >= CROSS_ENTER_CYCLES) {
                fsm->state = TRACK_CROSS;
                fsm->state_counter = 0;
                fsm->confirm_threshold = 0;
            }
        } else if (abs_err > ERROR_CURVE_ENTER) {
            fsm->state_counter++;
            if (fsm->state_counter >= CURVE_ENTER_CYCLES) {
                fsm->state = TRACK_CURVE;
                fsm->state_counter = 0;
                fsm->confirm_threshold = CURVE_EXIT_CYCLES;
            }
        } else {
            fsm->state_counter = 0;
        }
        break;

    case TRACK_CURVE:
        if (line_lost) {
            fsm->state_counter++;
            if (fsm->state_counter >= LOST_ENTER_CYCLES) {
                fsm->state = TRACK_LOST;
                fsm->state_counter = 0;
                fsm->confirm_threshold = LOST_EXIT_CYCLES;
            }
        } else if (all_black) {
            fsm->state = TRACK_CROSS;
            fsm->state_counter = 0;
            fsm->confirm_threshold = 0;
        } else if (abs_err < ERROR_CURVE_EXIT) {
            fsm->state_counter++;
            if (fsm->state_counter >= fsm->confirm_threshold) {
                fsm->state = TRACK_STRAIGHT;
                fsm->state_counter = 0;
                fsm->confirm_threshold = CURVE_ENTER_CYCLES;
            }
        } else {
            fsm->state_counter = 0;
        }
        break;

    case TRACK_CROSS:
        fsm->state_counter++;
        if (!all_black || (fsm->state_counter > CROSS_TIMEOUT)) {
            fsm->state = TRACK_STRAIGHT;
            fsm->state_counter = 0;
            fsm->confirm_threshold = CURVE_ENTER_CYCLES;
        }
        break;

    case TRACK_LOST:
        if (!line_lost) {
            fsm->state_counter++;
            if (fsm->state_counter >= fsm->confirm_threshold) {
                fsm->state = TRACK_STRAIGHT;
                fsm->state_counter = 0;
                fsm->confirm_threshold = CURVE_ENTER_CYCLES;
            }
        } else {
            fsm->state_counter = 0;
        }
        break;
    }

    switch (fsm->state) {
    case TRACK_STRAIGHT:
        fsm->base_speed = fsm->straight_speed;
        break;
    case TRACK_CURVE:
        fsm->base_speed = fsm->curve_speed;
        break;
    case TRACK_CROSS:
        fsm->base_speed = fsm->cross_speed;
        break;
    case TRACK_LOST:
        fsm->base_speed = fsm->lost_speed;
        break;
    }

    *out_base_speed = fsm->base_speed;
}