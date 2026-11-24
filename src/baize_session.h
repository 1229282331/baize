#ifndef BAIZE_SESSION_H
#define BAIZE_SESSION_H

#include "baize_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BAIZE_SEND_STOP, BAIZE_WAIT_DISABLED, BAIZE_READY, BAIZE_WAIT_ENABLED, BAIZE_ACTIVE
} BaizeHostPhase;

typedef struct {
    BaizeReceiver rx;
    BaizeHostPhase phase;
    uint32_t status_timeout_ms;
    union {
        uint32_t last_wheel_rx_ms;
        uint32_t last_status_ms; /* v1.0.0 field name, same storage. */
    };
    union { /* Enabling and waiting for a restarted board are mutually exclusive. */
        uint32_t enable_sent_ms;
        uint32_t restart_sample_ms;
    };
    uint32_t wheel_sample_ms, imu_sample_ms; /* Independent board timestamps. */
    uint8_t last_drive_state, last_stop_reason;
    bool have_wheel_sample, have_imu_sample;
    bool target_pending, awaiting_restart_ack;
    int32_t target_mrad_s[BAIZE_WHEELS];
} BaizeSession;

/* On every port open, purge old input, reset the session, then send its first command. */
void baize_session_open(BaizeSession *session, uint32_t status_timeout_ms);
/* Returns a new decoded sample. Poll next() even when only duplicates or IMU frames arrive. */
bool baize_session_receive(BaizeSession *session, uint8_t byte, uint32_t now_ms, BaizeFrame *feedback);
void baize_session_stop(BaizeSession *session);
/* After a STOP acknowledgement, returns the ENABLE frame for immediate transmission. */
bool baize_session_enable(BaizeSession *session, uint32_t now_ms, BaizeFrame *command);
/* Queue the latest wheel targets in mrad/s. The board enforces its configured range. */
bool baize_session_target(BaizeSession *session, const int32_t target_mrad_s[BAIZE_WHEELS]);
/* Poll on each send cycle, including when the caller has supplied no new target. */
bool baize_session_next(BaizeSession *session, uint32_t now_ms, BaizeFrame *command);

#ifdef __cplusplus
}
#endif
#endif
