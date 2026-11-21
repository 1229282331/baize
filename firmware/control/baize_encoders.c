#include "baize_encoders.h"

#include <math.h>

/* Vendor defaults for M1 through M4, in protocol wheel order. */
enum { COUNTS_PER_REV = 144000, VELOCITY_PERIOD_MS = 5 };
static const int8_t forward_sign[BAIZE_WHEELS] = {-1, -1, -1, 1};
static const double mrad_per_count = 6283.185307179586 / COUNTS_PER_REV;

bool baize_encoders_sample(BaizeEncoders *encoders, const uint16_t counter[BAIZE_WHEELS],
                           uint32_t sample_ms)
{
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        int32_t delta = (uint16_t)(counter[i] - encoders->counter[i]);
        if (delta == 0) continue;
        if (delta > INT16_MAX) {
            delta -= 65536;
        }
        int64_t total = encoders->counts[i] + delta;
        encoders->counts[i] = total;
        encoders->angle_mrad[i] = forward_sign[i] * llround((double)total * mrad_per_count);
        encoders->window_counts[i] += delta;
        encoders->counter[i] = counter[i];
    }
    encoders->sample_ms = sample_ms;
    uint32_t elapsed = sample_ms - encoders->velocity_ms;
    if (elapsed < VELOCITY_PERIOD_MS) return false;

    double mrad_s_per_count = mrad_per_count * 1000.0 / elapsed;
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        encoders->velocity_mrad_s[i] = (int32_t)lround(
            forward_sign[i] * encoders->window_counts[i] * mrad_s_per_count);
        encoders->window_counts[i] = 0;
    }
    encoders->velocity_ms = sample_ms;
    return true;
}
