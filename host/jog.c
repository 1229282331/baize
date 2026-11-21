#include "jog.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

bool baize_jog_parse(BaizeJog *jog, char *const args[])
{
    errno = 0;
    unsigned long ms = strtoul(args[0], NULL, 10);
    if (errno || ms == 0 || ms > 2000 || strspn(args[0], "0123456789") != strlen(args[0])) return false;
    *jog = (BaizeJog){.duration_ms = (uint32_t)ms};
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        char *end;
        const char *arg = args[i + 1];
        errno = 0;
        double speed = strtod(arg, &end);
        if (errno || end == arg || *end || !(speed >= -2.0 && speed <= 2.0)) return false;
        jog->target_mrad_s[i] = (int32_t)(speed * 1000.0 + (speed < 0 ? -0.5 : 0.5));
    }
    return true;
}

static void stop(BaizeJog *jog, uint64_t now, bool failed)
{
    jog->failed |= failed;
    if (jog->phase == JOG_STOPPING || jog->phase == JOG_DONE) return;
    jog->phase = JOG_STOPPING;
    jog->phase_ms = now;
    jog->stop_sent = false;
}

void baize_jog_cancel(BaizeJog *jog, uint64_t now)
{
    stop(jog, now, true);
}

void baize_jog_status(BaizeJog *jog, const BaizeStatus *status, uint64_t now)
{
    jog->stop_ack = status->drive_state == BAIZE_DISABLED && status->stop_reason == BAIZE_REQUESTED_STOP;
    if (status->drive_state == BAIZE_FAULT) stop(jog, now, true);
    if (jog->phase == JOG_RUNNING && status->drive_state != BAIZE_ENABLED) stop(jog, now, true);
    if (jog->phase == JOG_STOPPING && jog->stop_sent && jog->stop_ack) jog->phase = JOG_DONE;
}

bool baize_jog_next(BaizeJog *jog, BaizeSession *session, uint64_t now, BaizeFrame *command)
{
    uint32_t tick = (uint32_t)now;
    if (jog->phase == JOG_SYNC) {
        if (now - jog->phase_ms >= 1000) {
            stop(jog, now, true);
        } else if (jog->stop_ack && baize_session_enable(session, tick, command)) {
            jog->phase = JOG_ENABLING;
            return true;
        } else if (session->phase == BAIZE_READY) {
            baize_session_stop(session); /* Wait for an explicit STOP acknowledgement. */
        }
    }
    if (jog->phase == JOG_ENABLING) {
        if (session->phase == BAIZE_ACTIVE) {
            jog->phase = JOG_RUNNING;
            jog->phase_ms = now;
        } else if (session->phase != BAIZE_WAIT_ENABLED) {
            stop(jog, now, true);
        }
    }
    if (jog->phase == JOG_RUNNING) {
        if (session->phase != BAIZE_ACTIVE || tick - session->last_status_ms >= session->status_timeout_ms) {
            stop(jog, now, true);
        } else if (now - jog->phase_ms >= jog->duration_ms) {
            stop(jog, now, false);
        } else {
            baize_session_target(session, jog->target_mrad_s);
        }
    }
    if (jog->phase == JOG_STOPPING) {
        if (now - jog->phase_ms >= 1000) {
            jog->phase = JOG_DONE;
            jog->failed = true;
            return false;
        }
        baize_session_stop(session);
    }
    if (!baize_session_next(session, tick, command)) return false;
    if (command->type == BAIZE_STOP && jog->phase != JOG_SYNC) {
        /* The session itself also requests STOP on loss of wheel feedback. */
        if (jog->phase != JOG_STOPPING) stop(jog, now, true);
        jog->stop_sent = true;
    }
    return true;
}
