#ifndef BAIZE_ENCODERS_H
#define BAIZE_ENCODERS_H

#include "baize_protocol.h"

typedef struct {
    uint16_t counter[BAIZE_WHEELS];
    volatile int64_t counts[BAIZE_WHEELS]; /* Raw timer counts, observable over SWD. */
    volatile int64_t angle_mrad[BAIZE_WHEELS]; /* Forward-positive wheel angles. */
    uint32_t sample_ms;
    uint32_t velocity_ms;
    int32_t window_counts[BAIZE_WHEELS];
    int32_t velocity_mrad_s[BAIZE_WHEELS];
} BaizeEncoders;

/* Start both timestamps at the initial counter[] reading, with zero totals.
 * Sample every millisecond, with fewer than 32768 counts per wheel between reads.
 * Returns true when angles and new window-averaged velocities share this timestamp. */
bool baize_encoders_sample(BaizeEncoders *encoders, const uint16_t counter[BAIZE_WHEELS],
                           uint32_t sample_ms);

#endif
