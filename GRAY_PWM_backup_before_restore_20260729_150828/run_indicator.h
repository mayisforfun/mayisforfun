#ifndef RUN_INDICATOR_H_
#define RUN_INDICATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "board_port.h"

/*
 * LED/buzzer indicator.
 * Rule for the race car: no light and no buzzer while the car is moving.
 * Start/stop only gives a short beep, then output is turned off again.
 */

typedef struct {
    uint8_t beep_ticks;
    uint8_t led_ticks;
    bool stopped_led_hold;
} RunIndicator;

#define INDICATOR_START_BEEP_TICKS  8U   /* about 80 ms */
#define INDICATOR_STOP_BEEP_TICKS   20U  /* about 200 ms */
#define INDICATOR_STOP_LED_HOLD     1

static inline void RunIndicator_init(RunIndicator *indicator)
{
    indicator->beep_ticks = 0;
    indicator->led_ticks = 0;
    indicator->stopped_led_hold = false;
    Board_setBuzzer(false);
    Board_setStatusLed(false);
}

static inline void RunIndicator_onStart(RunIndicator *indicator)
{
    indicator->beep_ticks = INDICATOR_START_BEEP_TICKS;
    indicator->led_ticks = 0;
    indicator->stopped_led_hold = false;
}

static inline void RunIndicator_onStop(RunIndicator *indicator)
{
    indicator->beep_ticks = INDICATOR_STOP_BEEP_TICKS;
    indicator->led_ticks = INDICATOR_STOP_BEEP_TICKS;
#if INDICATOR_STOP_LED_HOLD
    indicator->stopped_led_hold = true;
#else
    indicator->stopped_led_hold = false;
#endif
}

static inline void RunIndicator_update(RunIndicator *indicator, bool running)
{
    if (running) {
        /* During motion, always keep LED and buzzer off after the short start beep. */
        if (indicator->beep_ticks > 0U) {
            Board_setBuzzer(true);
            indicator->beep_ticks--;
        } else {
            Board_setBuzzer(false);
        }
        Board_setStatusLed(false);
        return;
    }

    if (indicator->beep_ticks > 0U) {
        Board_setBuzzer(true);
        indicator->beep_ticks--;
    } else {
        Board_setBuzzer(false);
    }

    if (indicator->led_ticks > 0U) {
        Board_setStatusLed(true);
        indicator->led_ticks--;
    } else {
        Board_setStatusLed(indicator->stopped_led_hold);
    }
}

#endif

