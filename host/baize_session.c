#include "baize_session.h"

#include <string.h>

void baize_session_open(BaizeSession *session, uint32_t status_timeout_ms)
{
    *session = (BaizeSession){ .phase = BAIZE_SEND_STOP, .status_timeout_ms = status_timeout_ms };
}

void baize_session_stop(BaizeSession *session)
{
    session->phase = BAIZE_SEND_STOP;
    session->target_pending = false;
}

static void check_timeout(BaizeSession *session, uint32_t now_ms)
{
    if (session->phase == BAIZE_SEND_STOP || session->phase == BAIZE_WAIT_DISABLED) return;
    bool stale = now_ms - session->last_status_ms >= session->status_timeout_ms;
    bool enable_timed_out = session->phase == BAIZE_WAIT_ENABLED &&
                            now_ms - session->enable_sent_ms >= session->status_timeout_ms;
    if (stale || enable_timed_out) baize_session_stop(session);
}

bool baize_session_receive(BaizeSession *session, uint8_t byte, uint32_t now_ms, BaizeFrame *feedback)
{
    if (!baize_receive(&session->rx, BAIZE_TO_HOST, byte, feedback)) return false;
    if (feedback->type != BAIZE_STATUS) return true;

    /* A late status cannot revive a target queued before the gap. */
    check_timeout(session, now_ms);
    session->last_status_ms = now_ms;
    switch (feedback->status.drive_state) {
    case BAIZE_DISABLED:
        if (session->phase == BAIZE_WAIT_DISABLED) session->phase = BAIZE_READY;
        if (session->phase == BAIZE_ACTIVE) baize_session_stop(session);
        break;
    case BAIZE_ENABLED:
        if (session->phase == BAIZE_WAIT_ENABLED) session->phase = BAIZE_ACTIVE;
        if (session->phase == BAIZE_READY) baize_session_stop(session);
        break;
    default: /* Fault or unrecognized state. */
        baize_session_stop(session);
        break;
    }
    return true;
}

bool baize_session_enable(BaizeSession *session, uint32_t now_ms, BaizeFrame *command)
{
    check_timeout(session, now_ms);
    if (session->phase != BAIZE_READY) return false;
    session->phase = BAIZE_WAIT_ENABLED;
    session->enable_sent_ms = now_ms;
    command->type = BAIZE_ENABLE;
    return true;
}

bool baize_session_target(BaizeSession *session, const int32_t target_mrad_s[BAIZE_WHEELS])
{
    if (session->phase != BAIZE_ACTIVE) return false;
    memcpy(session->target_mrad_s, target_mrad_s, sizeof session->target_mrad_s);
    session->target_pending = true;
    return true;
}

bool baize_session_next(BaizeSession *session, uint32_t now_ms, BaizeFrame *command)
{
    check_timeout(session, now_ms);
    if (session->phase == BAIZE_SEND_STOP || session->phase == BAIZE_WAIT_DISABLED) {
        command->type = BAIZE_STOP;
        session->phase = BAIZE_WAIT_DISABLED;
        return true;
    }
    if (session->phase == BAIZE_ACTIVE && session->target_pending) {
        command->type = BAIZE_TARGET;
        memcpy(command->target_mrad_s, session->target_mrad_s, sizeof command->target_mrad_s);
        session->target_pending = false;
        return true;
    }
    return false;
}
