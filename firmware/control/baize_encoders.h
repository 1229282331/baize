#ifndef BAIZE_ENCODERS_H
#define BAIZE_ENCODERS_H

#include "baize_protocol.h"

typedef struct {
    uint16_t counter[BAIZE_WHEELS];
    volatile int64_t counts[BAIZE_WHEELS]; /* Raw timer counts, observable over SWD. */
    volatile int64_t angle_mrad[BAIZE_WHEELS]; /* Forward-positive wheel angles. */
    uint32_t sample_ms;
} BaizeEncoders;

/* Start with zero totals and the initial hardware readings in counter[].
 * Each wheel must move fewer than 32768 counts between successive samples. */
void baize_encoders_sample(BaizeEncoders *encoders, const uint16_t counter[BAIZE_WHEELS],
                           uint32_t sample_ms);

#endif
