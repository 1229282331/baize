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
    uint32_t last_status_ms;
    uint32_t enable_sent_ms;
    bool target_pending;
    int32_t target_mrad_s[BAIZE_WHEELS];
} BaizeSession;

/* On every port open, purge old input, reset the session, then send its first command. */
void baize_session_open(BaizeSession *session, uint32_t status_timeout_ms);
bool baize_session_receive(BaizeSession *session, uint8_t byte, uint32_t now_ms, BaizeFrame *feedback);
void baize_session_stop(BaizeSession *session);
bool baize_session_enable(BaizeSession *session, uint32_t now_ms, BaizeFrame *command);
bool baize_session_target(BaizeSession *session, const int32_t target_mrad_s[BAIZE_WHEELS]);
/* Poll on each send cycle, including when the caller has supplied no new target. */
bool baize_session_next(BaizeSession *session, uint32_t now_ms, BaizeFrame *command);

#ifdef __cplusplus
}
#endif
#endif
