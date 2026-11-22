#include "jog.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

bool jog_parse(Jog *jog, char *const args[])
{
    errno = 0;
    unsigned long ms = strtoul(args[0], NULL, 10);
    if (errno || ms == 0 || ms > 2000 || strspn(args[0], "0123456789") != strlen(args[0])) return false;
    *jog = (Jog){.duration_ms = (uint32_t)ms};
    for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) {
        char *end;
        const char *arg = args[i + 1];
        errno = 0;
        double speed = strtod(arg, &end);
        if (errno || end == arg || *end || !(speed >= -2.0 && speed <= 2.0)) return false;
        jog->wheels.rad_s[i] = speed;
    }
    return true;
}

bool jog_stop(Jog *jog, baize_chassis *chassis, uint64_t now_ms, bool interrupted)
{
    jog->failed |= interrupted;
    if (jog->phase == JOG_STOPPING || jog->phase == JOG_DONE) return true;
    jog->phase = JOG_STOPPING;
    jog->phase_ms = now_ms;
    return chassis_request_stop(chassis);
}

bool jog_step(Jog *jog, baize_chassis *chassis, uint64_t now_ms)
{
    BaizeHostPhase phase = chassis_phase(chassis);
    switch (jog->phase) {
    case JOG_SYNC: {
        if (now_ms - jog->phase_ms >= 1000) return jog_stop(jog, chassis, now_ms, true);
        int sent = chassis_request_enable(chassis);
        if (sent > 0) jog->phase = JOG_ENABLING;
        return sent >= 0;
    }
    case JOG_ENABLING:
        if (phase == BAIZE_WAIT_ENABLED) return true;
        if (phase != BAIZE_ACTIVE) return jog_stop(jog, chassis, now_ms, true);
        jog->phase = JOG_RUNNING;
        jog->phase_ms = now_ms;
        /* fall through */
    case JOG_RUNNING:
        if (phase != BAIZE_ACTIVE) return jog_stop(jog, chassis, now_ms, true);
        if (now_ms - jog->phase_ms >= jog->duration_ms) return jog_stop(jog, chassis, now_ms, false);
        if (baize_chassis_write(chassis, &jog->wheels)) return true;
        return jog_stop(jog, chassis, now_ms, true);
    case JOG_STOPPING:
        if (phase == BAIZE_READY) jog->phase = JOG_DONE;
        else if (now_ms - jog->phase_ms >= 1000) {
            jog->failed = true;
            jog->phase = JOG_DONE;
        }
        return true;
    case JOG_DONE:
        return true;
    }
    return false;
}
