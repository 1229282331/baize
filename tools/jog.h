#ifndef JOG_H
#define JOG_H

#include "chassis_io.h"

typedef struct {
    enum { JOG_SYNC, JOG_ENABLING, JOG_RUNNING, JOG_STOPPING, JOG_DONE } phase;
    uint32_t duration_ms;
    baize_wheel_speeds wheels;
    uint64_t phase_ms;
    bool failed;
} Jog;

bool jog_parse(Jog *jog, char *const args[]);
bool jog_stop(Jog *jog, baize_chassis *chassis, uint64_t now_ms, bool interrupted);
bool jog_step(Jog *jog, baize_chassis *chassis, uint64_t now_ms);
#endif
