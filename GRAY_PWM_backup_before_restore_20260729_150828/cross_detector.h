#ifndef CROSS_DETECTOR_H_
#define CROSS_DETECTOR_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * Intersection detector.
 * It only answers one question: did the car enter a cross/intersection area?
 * It does not decide left turn, right turn, straight, or final stop.
 */

typedef struct {
    uint16_t count;          /* total confirmed intersections */
    uint8_t enter_ticks;     /* consecutive cross-like samples */
    uint8_t release_ticks;   /* consecutive non-cross samples after latch */
    bool latched;            /* true while the same intersection is active */
    bool event;              /* true for one control cycle when count++ */
    bool candidate;          /* current gray pattern looks like a cross */
} CrossDetector;

#define CROSS_GRAY_MASK          0x1FU
#define CROSS_ENTER_TICKS        2U   /* 2 cycles = about 20 ms */
#define CROSS_RELEASE_TICKS      8U   /* 8 cycles = about 80 ms */
#define CROSS_MIN_BLACK_SENSORS  4U

static inline uint8_t CrossDetector_popcount5(uint8_t bits)
{
    uint8_t n = 0;
    bits &= CROSS_GRAY_MASK;

    for (uint8_t i = 0; i < 5U; i++) {
        if ((bits & (1U << i)) != 0U) {
            n++;
        }
    }

    return n;
}

static inline bool CrossDetector_isCrossPattern(uint8_t gray_bits)
{
    uint8_t bits = gray_bits & CROSS_GRAY_MASK;
    uint8_t black_count = CrossDetector_popcount5(bits);

    /* all black is the clearest intersection/finish-line signal */
    if (bits == CROSS_GRAY_MASK) {
        return true;
    }

    /* A wide black band usually covers at least 4 sensors. */
    if (black_count >= CROSS_MIN_BLACK_SENSORS) {
        return true;
    }

    /* Center plus both sides means the sensor is over a wide junction. */
    if (((bits & (1U << 2)) != 0U) &&
        ((bits & ((1U << 0) | (1U << 1))) != 0U) &&
        ((bits & ((1U << 3) | (1U << 4))) != 0U)) {
        return true;
    }

    return false;
}

static inline void CrossDetector_init(CrossDetector *detector)
{
    detector->count = 0;
    detector->enter_ticks = 0;
    detector->release_ticks = 0;
    detector->latched = false;
    detector->event = false;
    detector->candidate = false;
}

static inline void CrossDetector_update(CrossDetector *detector,
                                        uint8_t gray_bits,
                                        bool enabled)
{
    bool candidate;

    detector->event = false;

    if (!enabled) {
        detector->enter_ticks = 0;
        detector->release_ticks = 0;
        detector->latched = false;
        detector->candidate = false;
        return;
    }

    candidate = CrossDetector_isCrossPattern(gray_bits);
    detector->candidate = candidate;

    if (!detector->latched) {
        if (candidate) {
            if (detector->enter_ticks < CROSS_ENTER_TICKS) {
                detector->enter_ticks++;
            }
            if (detector->enter_ticks >= CROSS_ENTER_TICKS) {
                detector->latched = true;
                detector->event = true;
                detector->count++;
                detector->release_ticks = 0;
            }
        } else {
            detector->enter_ticks = 0;
        }
    } else {
        if (!candidate) {
            if (detector->release_ticks < CROSS_RELEASE_TICKS) {
                detector->release_ticks++;
            }
            if (detector->release_ticks >= CROSS_RELEASE_TICKS) {
                detector->latched = false;
                detector->enter_ticks = 0;
                detector->release_ticks = 0;
            }
        } else {
            detector->release_ticks = 0;
        }
    }
}

#endif
