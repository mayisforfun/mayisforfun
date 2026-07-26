#include "track_fsm.h"

/*
 * Track state machine — determines track section from sensor data
 * and adjusts base_speed accordingly.
 *
 * All 5 sensors on black (0x1F) = cross / intersection.
 * |error| > 600 for 3 cycles  = entering a curve.
 * |error| < 300 for 5 cycles  = exiting a curve.
 * No line seen for 10 cycles  = line lost.
 */

#define ERROR_CURVE_ENTER  600
#define ERROR_CURVE_EXIT   300
#define CURVE_ENTER_CYCLES 3
#define CURVE_EXIT_CYCLES  5
#define LOST_ENTER_CYCLES  10
#define LOST_EXIT_CYCLES   3
#define CROSS_TIMEOUT      30

void TrackFSM_init(TrackFSM *fsm)
{
    fsm->state             = TRACK_STRAIGHT;
    fsm->base_speed        = 420;
    fsm->state_counter     = 0;
    fsm->confirm_threshold = CURVE_ENTER_CYCLES;
    fsm->straight_speed    = 550;
    fsm->curve_speed       = 300;
    fsm->cross_speed       = 250;
    fsm->lost_speed        = 200;
}

static int16_t abs_i16(int16_t v)
{
    return (v >= 0) ? v : -v;
}

void TrackFSM_update(TrackFSM *fsm, const LineFollowState *line,
                     uint16_t *out_base_speed)
{
    bool    all_black = (line->raw_sensor_bits == 0x1FU);
    bool    line_lost = !line->line_seen;
    int16_t abs_err   = abs_i16(line->error);

    switch (fsm->state) {

    case TRACK_STRAIGHT:
        if (line_lost) {
            fsm->state_counter++;
            if (fsm->state_counter >= LOST_ENTER_CYCLES) {
                fsm->state             = TRACK_LOST;
                fsm->state_counter     = 0;
                fsm->confirm_threshold = LOST_EXIT_CYCLES;
                *out_base_speed        = fsm->lost_speed;
                return;
            }
        } else {
            fsm->state_counter = 0;
        }

        if (all_black) {
            fsm->state             = TRACK_CROSS;
            fsm->state_counter     = 0;
            fsm->confirm_threshold = 0;
            *out_base_speed        = fsm->cross_speed;
            return;
        }

        if (abs_err > ERROR_CURVE_ENTER) {
            fsm->state_counter++;
            if (fsm->state_counter >= CURVE_ENTER_CYCLES) {
                fsm->state             = TRACK_CURVE;
                fsm->state_counter     = 0;
                fsm->confirm_threshold = CURVE_EXIT_CYCLES;
                *out_base_speed        = fsm->curve_speed;
                return;
            }
        } else {
            fsm->state_counter = 0;
        }

        *out_base_speed = fsm->straight_speed;
        break;

    case TRACK_CURVE:
        if (all_black) {
            fsm->state             = TRACK_CROSS;
            fsm->state_counter     = 0;
            fsm->confirm_threshold = 0;
            *out_base_speed        = fsm->cross_speed;
            return;
        }

        if (abs_err < ERROR_CURVE_EXIT) {
            fsm->state_counter++;
            if (fsm->state_counter >= fsm->confirm_threshold) {
                fsm->state             = TRACK_STRAIGHT;
                fsm->state_counter     = 0;
                fsm->confirm_threshold = CURVE_ENTER_CYCLES;
                *out_base_speed        = fsm->straight_speed;
                return;
            }
        } else {
            fsm->state_counter = 0;
        }

        *out_base_speed = fsm->curve_speed;
        break;

    case TRACK_CROSS:
        fsm->state_counter++;
        if (!all_black) {
            fsm->state             = TRACK_STRAIGHT;
            fsm->state_counter     = 0;
            fsm->confirm_threshold = CURVE_ENTER_CYCLES;
            *out_base_speed        = fsm->straight_speed;
            return;
        }
        if (fsm->state_counter > CROSS_TIMEOUT) {
            /* Stuck — force exit */
            fsm->state             = TRACK_STRAIGHT;
            fsm->state_counter     = 0;
            fsm->confirm_threshold = CURVE_ENTER_CYCLES;
            *out_base_speed        = fsm->straight_speed;
            return;
        }
        *out_base_speed = fsm->cross_speed;
        break;

    case TRACK_LOST:
        if (!line_lost) {
            fsm->state_counter++;
            if (fsm->state_counter >= fsm->confirm_threshold) {
                fsm->state             = TRACK_STRAIGHT;
                fsm->state_counter     = 0;
                fsm->confirm_threshold = CURVE_ENTER_CYCLES;
                *out_base_speed        = fsm->straight_speed;
                return;
            }
        } else {
            fsm->state_counter = 0;
        }
        *out_base_speed = fsm->lost_speed;
        break;
    }
}
