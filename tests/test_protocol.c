#include "baize_protocol.h"

#undef NDEBUG
#undef NDEBUG
#include <assert.h>
#include <string.h>

/* Published wire bytes in docs/base-interface.md. */
static const uint8_t stop[] = { 5, 1, 1, 0x1f, 0x3e, 0 };
static const uint8_t enable[] = { 5, 1, 2, 0x7c, 0x0e, 0 };
static const uint8_t target[] = {
    0x09, 0x01, 0x03, 0x18, 0xfc, 0xff, 0xff, 0xe8, 0x03, 0x01, 0x07,
    0x18, 0xfc, 0xff, 0xff, 0xe8, 0x03, 0x01, 0x03, 0xcc, 0x7c, 0x00
};
static const uint8_t status[] = {
    0x05, 0x01, 0x81, 0xe8, 0x03, 0x01, 0x02, 0x01, 0x0d, 0xd0, 0x39, 0x18, 0xfc, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xe8, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0b, 0x18, 0xfc, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xe8, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x07, 0x7c, 0xfc,
    0xff, 0xff, 0x84, 0x03, 0x01, 0x07, 0x7c, 0xfc, 0xff, 0xff, 0x84, 0x03, 0x01, 0x03, 0xfa,
    0x19, 0x00
};
static const uint8_t imu[] = {
    0x05, 0x01, 0x82, 0xfc, 0x03, 0x01, 0x02, 0x64, 0x01, 0x01, 0x07, 0x38, 0xff, 0xff, 0xff,
    0x4f, 0x26, 0x01, 0x01, 0x01, 0x01, 0x01, 0x07, 0x06, 0xff, 0xff, 0xff, 0xf4, 0x01, 0x01,
    0x03, 0x3f, 0xe6, 0x00
};

static void check_vector(BaizeDirection direction, const BaizeFrame *sample,
                         const uint8_t *expected, size_t length)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    assert(baize_encode(sample, wire) == length);
    assert(memcmp(wire, expected, length) == 0);

    BaizeDirection other = direction == BAIZE_TO_BASE ? BAIZE_TO_HOST : BAIZE_TO_BASE;
    BaizeReceiver rx = {0}, wrong_port = {0};
    BaizeFrame decoded = {0}, ignored = {0};
    for (size_t i = 0; i < length; ++i) {
        assert(baize_receive(&rx, direction, expected[i], &decoded) == (i + 1 == length));
        assert(!baize_receive(&wrong_port, other, expected[i], &ignored));
    }
    assert(decoded.type == sample->type);
    assert(baize_encode(&decoded, wire) == length);
    assert(memcmp(wire, expected, length) == 0);
}

static void published_vectors(void)
{
    check_vector(BAIZE_TO_BASE, &(BaizeFrame){ .type = BAIZE_STOP }, stop, sizeof stop);
    check_vector(BAIZE_TO_BASE, &(BaizeFrame){ .type = BAIZE_ENABLE }, enable, sizeof enable);
    check_vector(BAIZE_TO_BASE, &(BaizeFrame){
        .type = BAIZE_TARGET, .target_mrad_s = {-1000, 1000, -1000, 1000}
    }, target, sizeof target);
    check_vector(BAIZE_TO_HOST, &(BaizeFrame){ .type = BAIZE_STATUS, .status = {
        .sample_ms = 1000, .drive_state = BAIZE_ENABLED, .stop_reason = BAIZE_RUNNING,
        .supply_mv = 14800, .angle_mrad = {-1000, 1000, -1000, 1000},
        .velocity_mrad_s = {-900, 900, -900, 900}
    } }, status, sizeof status);
    check_vector(BAIZE_TO_HOST, &(BaizeFrame){ .type = BAIZE_IMU, .imu = {
        .sample_ms = 1020, .acceleration_mm_s2 = {100, -200, 9807}, .gyro_mrad_s = {0, -250, 500}
    } }, imu, sizeof imu);
}

static void invalid_then_valid(BaizeReceiver *rx, const uint8_t *bad, size_t length)
{
    BaizeFrame output = { .type = BAIZE_IMU };
    for (size_t i = 0; i < length; ++i) {
        assert(!baize_receive(rx, BAIZE_TO_BASE, bad[i], &output));
    }
    assert(output.type == BAIZE_IMU);
    for (size_t i = 0; i < sizeof enable; ++i) {
        assert(baize_receive(rx, BAIZE_TO_BASE, enable[i], &output) == (i + 1 == sizeof enable));
    }
    assert(output.type == BAIZE_ENABLE);
}

static void stream_recovery(void)
{
    BaizeReceiver rx = {0};
    const uint8_t empty[] = {0, 1, 0};
    const uint8_t bad_cobs[] = {8, 1, 2, 0};
    const uint8_t bad_crc[] = {5, 1, 2, 0x7d, 0x0e, 0};
    /* These three frames have correct CRCs for their invalid headers or length. */
    const uint8_t version[] = {5, 2, 2, 0x2f, 0x5b, 0};
    const uint8_t type[] = {5, 1, 4, 0xba, 0x6e, 0};
    const uint8_t length[] = {5, 1, 3, 0x5d, 0x1e, 0};
    invalid_then_valid(&rx, empty, sizeof empty);
    invalid_then_valid(&rx, bad_cobs, sizeof bad_cobs);
    invalid_then_valid(&rx, bad_crc, sizeof bad_crc);
    invalid_then_valid(&rx, version, sizeof version);
    invalid_then_valid(&rx, type, sizeof type);
    invalid_then_valid(&rx, length, sizeof length);

    uint8_t oversized[BAIZE_WIRE_MAX + sizeof enable];
    memset(oversized, 1, BAIZE_WIRE_MAX);
    memcpy(oversized + BAIZE_WIRE_MAX, enable, sizeof enable);
    invalid_then_valid(&rx, oversized, sizeof oversized);

    BaizeFrame frame;
    for (size_t i = 0; i < sizeof stop - 1; ++i) {
        assert(!baize_receive(&rx, BAIZE_TO_BASE, stop[i], &frame));
    }
    assert(baize_receive(&rx, BAIZE_TO_BASE, 0, &frame) && frame.type == BAIZE_STOP);
}

static void integer_boundaries(void)
{
    BaizeFrame sample = { .type = BAIZE_STATUS, .status = {
        .sample_ms = UINT32_MAX, .supply_mv = UINT16_MAX,
        .angle_mrad = {INT64_MIN, INT64_MAX, INT64_C(0x0102030405060708), -1},
        .velocity_mrad_s = {INT32_MIN, INT32_MAX, 0x01020304, -1}
    } };
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(&sample, wire);
    assert(length == BAIZE_WIRE_MAX);
    BaizeReceiver rx = {0};
    BaizeFrame frame;
    for (size_t i = 0; i < length; ++i) {
        assert(baize_receive(&rx, BAIZE_TO_HOST, wire[i], &frame) == (i + 1 == length));
    }
    assert(frame.status.sample_ms == UINT32_MAX && frame.status.supply_mv == UINT16_MAX);
    assert(memcmp(frame.status.angle_mrad, sample.status.angle_mrad, sizeof sample.status.angle_mrad) == 0);
    assert(memcmp(frame.status.velocity_mrad_s, sample.status.velocity_mrad_s, sizeof sample.status.velocity_mrad_s) == 0);
    assert(baize_encode(&(BaizeFrame){ .type = 0xff }, wire) == 0);
}

int main(void)
{
    published_vectors();
    stream_recovery();
    integer_boundaries();
    return 0;
}
