#include "baize_encoders.h"

void baize_encoders_sample(BaizeEncoders *encoders, const uint16_t counter[BAIZE_WHEELS],
                           uint32_t sample_ms)
{
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        int32_t delta = (uint16_t)(counter[i] - encoders->counter[i]);
        if (delta > INT16_MAX) {
            delta -= 65536;
        }
        encoders->counts[i] += delta;
        encoders->counter[i] = counter[i];
    }
    encoders->sample_ms = sample_ms;
}
