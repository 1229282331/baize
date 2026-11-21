#ifndef BAIZE_PROTOCOL_H
#define BAIZE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { BAIZE_WHEELS = 4, BAIZE_WIRE_MAX = 62 };

enum {
    BAIZE_STOP = 0x01,
    BAIZE_ENABLE = 0x02,
    BAIZE_TARGET = 0x03,
    BAIZE_STATUS = 0x81,
    BAIZE_IMU = 0x82
};

enum { BAIZE_DISABLED = 0, BAIZE_ENABLED = 1, BAIZE_FAULT = 2 };
enum {
    BAIZE_RUNNING = 0,
    BAIZE_RESET = 1,
    BAIZE_REQUESTED_STOP = 2,
    BAIZE_TIMEOUT = 3,
    BAIZE_DRIVE_FAULT = 4
};

typedef struct {
    uint32_t sample_ms;
    uint8_t drive_state;
    uint8_t stop_reason;
    uint16_t supply_mv;
    int64_t angle_mrad[BAIZE_WHEELS];
    int32_t velocity_mrad_s[BAIZE_WHEELS];
} BaizeStatus;

typedef struct {
    uint32_t sample_ms;
    int32_t acceleration_mm_s2[3];
    int32_t gyro_mrad_s[3];
} BaizeImu;

typedef struct {
    uint8_t type;
    union {
        int32_t target_mrad_s[BAIZE_WHEELS];
        BaizeStatus status;
        BaizeImu imu;
    };
} BaizeFrame;

typedef enum { BAIZE_TO_BASE, BAIZE_TO_HOST } BaizeDirection;

/* Zero-initialize at the beginning of each stream. */
typedef struct {
    uint8_t encoded[BAIZE_WIRE_MAX - 1];
    size_t length;
    bool discarding;
} BaizeReceiver;

/* Returns the encoded length including the delimiter, or 0 for an unknown type. */
size_t baize_encode(const BaizeFrame *frame, uint8_t wire[BAIZE_WIRE_MAX]);

/* Writes frame only at the delimiter of a complete, valid frame for this direction. */
bool baize_receive(BaizeReceiver *rx, BaizeDirection direction, uint8_t byte, BaizeFrame *frame);

#ifdef __cplusplus
}
#endif
#endif
