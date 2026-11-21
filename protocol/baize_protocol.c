#include "baize_protocol.h"

#include <string.h>

/* Raw frame lengths include version, type and CRC. */
static size_t raw_length(uint8_t type)
{
    switch (type) {
    case BAIZE_STOP:
    case BAIZE_ENABLE: return 4;
    case BAIZE_TARGET: return 20;
    case BAIZE_STATUS: return 60;
    case BAIZE_IMU:    return 32;
    default:           return 0;
    }
}

static uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc & 0x8000u) ? (crc << 1) ^ 0x1021 : crc << 1);
        }
    }
    return crc;
}

static void put_le(uint8_t *out, uint64_t value, size_t bytes)
{
    for (size_t i = 0; i < bytes; ++i) {
        out[i] = (uint8_t)value;
        value >>= 8;
    }
}

static uint64_t get_le(const uint8_t *in, size_t bytes)
{
    uint64_t value = 0;
    while (bytes > 0) {
        value = (value << 8) | in[--bytes];
    }
    return value;
}

static int32_t get_i32(const uint8_t *in)
{
    uint32_t bits = (uint32_t)get_le(in, 4);
    int32_t value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static int64_t get_i64(const uint8_t *in)
{
    uint64_t bits = get_le(in, 8);
    int64_t value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

static void pack(const BaizeFrame *frame, uint8_t *p)
{
    switch (frame->type) {
    case BAIZE_TARGET:
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            put_le(p + 4*i, (uint32_t)frame->target_mrad_s[i], 4);
        }
        break;
    case BAIZE_STATUS:
        put_le(p, frame->status.sample_ms, 4);
        p[4] = frame->status.drive_state;
        p[5] = frame->status.stop_reason;
        put_le(p + 6, frame->status.supply_mv, 2);
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            put_le(p + 8 + 8*i, (uint64_t)frame->status.angle_mrad[i], 8);
            put_le(p + 40 + 4*i, (uint32_t)frame->status.velocity_mrad_s[i], 4);
        }
        break;
    case BAIZE_IMU:
        put_le(p, frame->imu.sample_ms, 4);
        for (size_t i = 0; i < 3; ++i) {
            put_le(p + 4 + 4*i, (uint32_t)frame->imu.acceleration_mm_s2[i], 4);
            put_le(p + 16 + 4*i, (uint32_t)frame->imu.gyro_mrad_s[i], 4);
        }
        break;
    }
}

static void unpack(BaizeFrame *frame, const uint8_t *p)
{
    switch (frame->type) {
    case BAIZE_TARGET:
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            frame->target_mrad_s[i] = get_i32(p + 4*i);
        }
        break;
    case BAIZE_STATUS:
        frame->status.sample_ms = (uint32_t)get_le(p, 4);
        frame->status.drive_state = p[4];
        frame->status.stop_reason = p[5];
        frame->status.supply_mv = (uint16_t)get_le(p + 6, 2);
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            frame->status.angle_mrad[i] = get_i64(p + 8 + 8*i);
            frame->status.velocity_mrad_s[i] = get_i32(p + 40 + 4*i);
        }
        break;
    case BAIZE_IMU:
        frame->imu.sample_ms = (uint32_t)get_le(p, 4);
        for (size_t i = 0; i < 3; ++i) {
            frame->imu.acceleration_mm_s2[i] = get_i32(p + 4 + 4*i);
            frame->imu.gyro_mrad_s[i] = get_i32(p + 16 + 4*i);
        }
        break;
    }
}

size_t baize_encode(const BaizeFrame *frame, uint8_t wire[BAIZE_WIRE_MAX])
{
    uint8_t raw[BAIZE_WIRE_MAX - 2];
    size_t length = raw_length(frame->type);
    if (length == 0) return 0;
    raw[0] = 1;
    raw[1] = frame->type;
    pack(frame, raw + 2);
    put_le(raw + length - 2, crc16(raw, length - 2), 2);

    /* Every v1 frame fits within a single 254-byte COBS run. */
    size_t code = 0;
    size_t next = 1;
    for (size_t i = 0; i < length; ++i) {
        if (raw[i] == 0) {
            wire[code] = (uint8_t)(next - code);
            code = next++;
        } else {
            wire[next++] = raw[i];
        }
    }
    wire[code] = (uint8_t)(next - code);
    wire[next++] = 0;
    return next;
}

static bool decode(const uint8_t *encoded, size_t size, BaizeDirection direction, BaizeFrame *frame)
{
    uint8_t raw[BAIZE_WIRE_MAX - 2];
    size_t length = 0;
    size_t pos = 0;
    while (pos < size) {
        size_t count = encoded[pos++] - 1u;
        if (count > size - pos) return false;
        memcpy(raw + length, encoded + pos, count);
        length += count;
        pos += count;
        if (pos < size) raw[length++] = 0;
    }
    if (length < 4 || raw[0] != 1 || raw_length(raw[1]) != length) return false;
    BaizeDirection intended = raw[1] < 0x80 ? BAIZE_TO_BASE : BAIZE_TO_HOST;
    if (direction != intended || get_le(raw + length - 2, 2) != crc16(raw, length - 2)) return false;

    frame->type = raw[1];
    unpack(frame, raw + 2);
    return true;
}

bool baize_receive(BaizeReceiver *rx, BaizeDirection direction, uint8_t byte, BaizeFrame *frame)
{
    if (byte == 0) {
        bool valid = !rx->discarding && decode(rx->encoded, rx->length, direction, frame);
        rx->length = 0;
        rx->discarding = false;
        return valid;
    }
    if (rx->length < sizeof rx->encoded) {
        rx->encoded[rx->length++] = byte;
    } else {
        rx->discarding = true;
    }
    return false;
}
