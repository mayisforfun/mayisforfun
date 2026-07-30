#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

/*
 * Encoder module.
 * The get functions return delta ticks since the last call, not total ticks.
 * In the current main loop, one call happens about every 10 ms.
 */
void Encoder_init(void);
int32_t Encoder_getLeftTicks(void);
int32_t Encoder_getRightTicks(void);
void Encoder_clearDeltas(void);

#endif
