#ifndef BAIZE_JOG_H
#define BAIZE_JOG_H

#include "baize_session.h"

typedef struct {
    enum { JOG_SYNC, JOG_ENABLING, JOG_RUNNING, JOG_STOPPING, JOG_DONE } phase;
    uint32_t duration_ms;
    int32_t target_mrad_s[BAIZE_WHEELS];
    uint64_t phase_ms;
    bool stop_ack, stop_sent, failed;
} BaizeJog;

/* args: duration in ms, then four wheel velocities in rad/s. */
bool baize_jog_parse(BaizeJog *jog, char *const args[]);
void baize_jog_cancel(BaizeJog *jog, uint64_t now);
void baize_jog_status(BaizeJog *jog, const BaizeStatus *status, uint64_t now);
bool baize_jog_next(BaizeJog *jog, BaizeSession *session, uint64_t now, BaizeFrame *command);

#endif
