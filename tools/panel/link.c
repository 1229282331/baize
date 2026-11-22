/* The panel worker owns the handle and records every feedback sample. */
#include "jog.h"

#include <math.h>
#include <stdlib.h>

typedef struct {
    baize_chassis *chassis;
    Jog jog;
    bool jogging;
    uint64_t next_send_ms, last_wheel_rx_ms;
} PanelLink;

enum { SYNCING, READY, BUSY, FINISHED, INTERRUPTED };

size_t panel_frame_size(void) { return sizeof(FeedbackSample); }

PanelLink *panel_open(const char *name)
{
    PanelLink *link = calloc(1, sizeof *link);
    if (!link) return NULL;
    link->chassis = chassis_connect(name);
    if (!link->chassis) {
        free(link);
        return NULL;
    }
    link->last_wheel_rx_ms = monotonic_ms();
    return link;
}

int panel_start(PanelLink *link, uint32_t duration_ms, const double target[BAIZE_WHEEL_COUNT])
{
    if (link->jogging || !duration_ms || duration_ms > 2000) return 0;
    Jog jog = {.phase = JOG_ENABLING, .duration_ms = duration_ms};
    bool moving = false;
    for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) {
        if (!(target[i] >= -2 && target[i] <= 2)) return 0;
        moving |= lround(target[i] * 1000.0) != 0;
        jog.wheels.rad_s[i] = target[i];
    }
    if (!moving) return 0;
    int sent = chassis_request_enable(link->chassis);
    if (sent != 1) return sent;
    link->jog = jog;
    link->jogging = true;
    link->next_send_ms = 0;
    return 1;
}

void panel_stop(PanelLink *link)
{
    if (link->jogging) jog_stop(&link->jog, link->chassis, monotonic_ms(), true);
    else chassis_request_stop(link->chassis);
    link->next_send_ms = 0;
}

int panel_poll(PanelLink *link, FeedbackSample samples[FEEDBACK_BATCH], int *state)
{
    uint64_t now_ms = monotonic_ms();
    if (link->jogging && now_ms >= link->next_send_ms) {
        if (!jog_step(&link->jog, link->chassis, now_ms)) return -1;
        link->next_send_ms = now_ms + 20;
    }
    int count = chassis_poll(link->chassis, samples);
    if (count < 0) return -1;
    now_ms = monotonic_ms();
    for (int i = 0; i < count; ++i) {
        if (samples[i].kind == BAIZE_STATUS) link->last_wheel_rx_ms = now_ms;
    }
    if (now_ms - link->last_wheel_rx_ms >= 1000) return -2;
    if (link->jogging && link->jog.phase == JOG_DONE) {
        *state = link->jog.failed ? INTERRUPTED : FINISHED;
        link->jogging = false;
    } else if (link->jogging) {
        *state = BUSY;
    } else {
        *state = chassis_phase(link->chassis) == BAIZE_READY ? READY : SYNCING;
    }
    return count;
}

void panel_close(PanelLink *link)
{
    baize_chassis_close(link->chassis);
    free(link);
}
