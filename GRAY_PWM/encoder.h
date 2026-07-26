#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void Encoder_init(void);
int32_t Encoder_getLeftTicks(void);
int32_t Encoder_getRightTicks(void);
void Encoder_clearDeltas(void);

#endif
