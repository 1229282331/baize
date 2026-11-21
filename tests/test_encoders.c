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

static void velocity_window_and_clock_wrap(void)
{
    BaizeEncoders encoders = {.sample_ms = UINT32_MAX - 2, .velocity_ms = UINT32_MAX - 2};
    const uint16_t first[] = {65535, 1, 0, 1};
    const uint16_t second[] = {65534, 2, 0, 2};
    assert(!baize_encoders_sample(&encoders, first, UINT32_MAX));
    assert(baize_encoders_sample(&encoders, second, 2));
    /* Two counts in 5 ms remain visible below the 1 mrad angle resolution. */
    assert(encoders.angle_mrad[0] == 0);
    assert(encoders.velocity_mrad_s[0] == 17 && encoders.velocity_mrad_s[1] == -17);
    assert(encoders.velocity_mrad_s[2] == 0 && encoders.velocity_mrad_s[3] == 17);
    const uint16_t zero[] = {0, 0, 0, 0};
    assert(baize_encoders_sample(&encoders, zero, 12));
    assert(encoders.velocity_mrad_s[0] == -9 && encoders.velocity_mrad_s[1] == 9);
    assert(baize_encoders_sample(&encoders, zero, 17));
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) assert(encoders.velocity_mrad_s[i] == 0);
}

int main(void)
{
    reverse_across_wrap();
    accumulate_past_32_bits();
    vendor_wheel_angles();
    velocity_window_and_clock_wrap();
    return 0;
}
