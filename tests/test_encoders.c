#include "baize_encoders.h"

#undef NDEBUG
#include <assert.h>

static void reverse_across_wrap(void)
{
    const uint16_t start[BAIZE_WHEELS] = {65534, 1, 1234, 32766};
    const uint16_t moved[BAIZE_WHEELS] = {2, 65534, 1234, 32769};
    const int64_t expected[BAIZE_WHEELS] = {4, -3, 0, 3};
    BaizeEncoders encoders = {0};
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        encoders.counter[i] = start[i];
    }

    baize_encoders_sample(&encoders, moved, UINT32_MAX);
    assert(encoders.sample_ms == UINT32_MAX);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        assert(encoders.counts[i] == expected[i]);
    }
    baize_encoders_sample(&encoders, moved, 0);
    assert(encoders.sample_ms == 0);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        assert(encoders.counts[i] == expected[i]);
    }
    baize_encoders_sample(&encoders, start, 1);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        assert(encoders.counts[i] == 0);
    }
}

static void accumulate_past_32_bits(void)
{
    BaizeEncoders encoders = {0};
    for (uint32_t step = 1; step <= 65540; ++step) {
        int64_t ticks = (int64_t)step * INT16_MAX;
        uint16_t counter[BAIZE_WHEELS] = {(uint16_t)ticks, (uint16_t)(-ticks), 0, 0};
        baize_encoders_sample(&encoders, counter, step);
        assert(encoders.counts[0] == ticks);
        assert(encoders.counts[1] == -ticks);
    }
    assert(encoders.counts[0] > INT32_MAX && encoders.counts[1] < INT32_MIN);
}

static void vendor_wheel_angles(void)
{
    /* Six steps of 24000 raw counts make one output revolution. */
    for (int direction = -1; direction <= 1; direction += 2) {
        BaizeEncoders encoders = {0};
        for (int step = 1; step <= 6; ++step) {
            int ticks = direction * step * 24000;
            uint16_t counter[BAIZE_WHEELS] = {
                (uint16_t)(-ticks), (uint16_t)(-ticks), (uint16_t)(-ticks), (uint16_t)ticks
            };
            baize_encoders_sample(&encoders, counter, (uint32_t)step);
        }
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            assert(encoders.angle_mrad[i] == direction * 6283);
        }
    }
}

int main(void)
{
    reverse_across_wrap();
    accumulate_past_32_bits();
    vendor_wheel_angles();
    return 0;
}
