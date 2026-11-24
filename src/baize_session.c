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
    bool stale = now_ms - session->last_wheel_rx_ms >= session->status_timeout_ms;
    bool enable_timed_out = session->phase == BAIZE_WAIT_ENABLED &&
                            now_ms - session->enable_sent_ms >= session->status_timeout_ms;
    if (stale || enable_timed_out) baize_session_stop(session);
}

static bool later(uint32_t tick, uint32_t previous)
{
    uint32_t delta = tick - previous;
    return delta != 0 && delta <= INT32_MAX;
}

bool baize_session_receive(BaizeSession *session, uint8_t byte, uint32_t now_ms, BaizeFrame *feedback)
{
    if (!baize_receive(&session->rx, BAIZE_TO_HOST, byte, feedback)) return false;
    if (feedback->type == BAIZE_MAINTENANCE_REPLY) return false;
    if (feedback->type == BAIZE_IMU) {
        uint32_t sample_ms = feedback->imu.sample_ms;
        if (session->awaiting_restart_ack ||
            (session->have_imu_sample && !later(sample_ms, session->imu_sample_ms))) return false;
        session->have_imu_sample = true;
        session->imu_sample_ms = sample_ms;
        return true;
    }

    /* A late status cannot revive a target queued before the gap. */
    check_timeout(session, now_ms);
    const BaizeStatus *status = &feedback->status;
    uint32_t tick = status->sample_ms;
    if (session->have_wheel_sample && !later(tick, session->wheel_sample_ms)) {
        if (tick == session->wheel_sample_ms && status->drive_state == session->last_drive_state &&
            status->stop_reason == session->last_stop_reason) return false;
        /* An old frame cancels motion. Only an advancing post-STOP acknowledgement
           can establish a younger clock; small reorders keep the high watermark. */
        bool restart_ack = session->awaiting_restart_ack && session->phase == BAIZE_WAIT_DISABLED &&
                           later(tick, session->restart_sample_ms) && tick != session->wheel_sample_ms &&
                           status->drive_state == BAIZE_DISABLED && status->stop_reason == BAIZE_REQUESTED_STOP;
        if (!restart_ack) {
            uint64_t possible_uptime = (uint32_t)(now_ms - session->last_wheel_rx_ms)
                                       + (uint64_t)session->status_timeout_ms;
            if (!session->awaiting_restart_ack && tick <= possible_uptime &&
                (status->stop_reason == BAIZE_RESET ||
                 session->wheel_sample_ms - tick >= session->status_timeout_ms)) {
                session->awaiting_restart_ack = true;
                session->restart_sample_ms = tick;
                baize_session_stop(session);
            }
            if (session->phase != BAIZE_WAIT_DISABLED) baize_session_stop(session);
            return false;
        }
    }
    if (session->awaiting_restart_ack || (status->stop_reason == BAIZE_RESET &&
        session->have_wheel_sample && session->last_stop_reason != BAIZE_RESET))
        session->have_imu_sample = false;
    session->awaiting_restart_ack = false;
    session->have_wheel_sample = true;
    session->wheel_sample_ms = tick;
    session->last_drive_state = status->drive_state;
    session->last_stop_reason = status->stop_reason;
    session->last_wheel_rx_ms = now_ms;
    switch (status->drive_state) {
    case BAIZE_DISABLED:
        if (session->phase == BAIZE_WAIT_DISABLED) {
            if (status->stop_reason == BAIZE_REQUESTED_STOP) session->phase = BAIZE_READY;
        } else if (session->phase == BAIZE_ACTIVE || status->stop_reason != BAIZE_REQUESTED_STOP) {
            baize_session_stop(session);
        }
        break;
    case BAIZE_ENABLED:
        if (session->phase == BAIZE_READY || status->stop_reason != BAIZE_RUNNING) {
            baize_session_stop(session);
        } else if (session->phase == BAIZE_WAIT_ENABLED) {
            session->phase = BAIZE_ACTIVE;
        }
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
