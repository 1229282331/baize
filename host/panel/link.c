/* Local desktop panel adapter. One I/O thread owns a link for its entire lifetime. */
#include "baize_session.h"
#include "jog.h"
#include "serial_port.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    SerialPort port;
    BaizeSession session;
    BaizeJog jog;
    bool jogging;
    uint64_t next_send_ms, last_wheel_ms;
} PanelLink;

enum { SYNCING, READY, BUSY, FINISHED, INTERRUPTED };

size_t panel_frame_size(void) { return sizeof(BaizeFrame); }

PanelLink *panel_open(const char *name)
{
    PanelLink *link = calloc(1, sizeof *link);
    if (!link) return NULL;
    link->port = serial_open(name);
    if (link->port == SERIAL_INVALID) {
        free(link);
        return NULL;
    }
    baize_session_open(&link->session, 250);
    uint64_t now = monotonic_ms();
    BaizeFrame stop;
    baize_session_next(&link->session, (uint32_t)now, &stop);
    if (!serial_write_frame(link->port, &stop)) {
        serial_close(link->port);
        free(link);
        return NULL;
    }
    link->last_wheel_ms = now;
    link->next_send_ms = now + 20;
    return link;
}

/* 1 started, 0 rejected, -1 write failure. A new click requires a fresh STOP acknowledgement. */
int panel_start(PanelLink *link, uint32_t duration_ms, const int32_t target[BAIZE_WHEELS])
{
    if (link->jogging || !duration_ms || duration_ms > 2000) return 0;
    bool moving = false;
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        if (target[i] < -2000 || target[i] > 2000) return 0;
        moving |= target[i] != 0;
    }
    uint64_t now = monotonic_ms();
    BaizeFrame command;
    if (!moving || !baize_session_enable(&link->session, (uint32_t)now, &command)) return 0;
    if (!serial_write_frame(link->port, &command)) return -1;
    link->jog = (BaizeJog){.phase = JOG_ENABLING, .phase_ms = now, .duration_ms = duration_ms};
    memcpy(link->jog.target_mrad_s, target, sizeof link->jog.target_mrad_s);
    link->jogging = true;
    link->next_send_ms = now;
    return 1;
}

void panel_stop(PanelLink *link)
{
    if (link->jogging) baize_jog_cancel(&link->jog, monotonic_ms());
    baize_session_stop(&link->session);
    link->next_send_ms = 0;
}

/* At most eight feedback frames can finish in 256 bytes (shortest frame is 34 bytes). */
int panel_poll(PanelLink *link, BaizeFrame frames[8], int *state)
{
    uint64_t now = monotonic_ms();
    bool done = false;
    if (now >= link->next_send_ms) {
        BaizeFrame command;
        bool send = link->jogging ? baize_jog_next(&link->jog, &link->session, now, &command)
                                  : baize_session_next(&link->session, (uint32_t)now, &command);
        if (send && !serial_write_frame(link->port, &command)) return -1;
        done = link->jogging && link->jog.phase == JOG_DONE;
        if (done) link->jogging = false;
        link->next_send_ms = now + 20;
    }

    uint8_t data[256];
    int size = serial_read(link->port, data, sizeof data);
    if (size < 0) return -1;
    now = monotonic_ms();
    int count = 0;
    for (int i = 0; i < size; ++i) {
        BaizeFrame frame;
        if (!baize_session_receive(&link->session, data[i], (uint32_t)now, &frame)) continue;
        if (frame.type == BAIZE_STATUS) {
            link->last_wheel_ms = now;
            if (link->jogging) baize_jog_status(&link->jog, &frame.status, now);
        }
        frames[count++] = frame;
    }
    if (now - link->last_wheel_ms >= 1000) return -2;
    if (done) *state = link->jog.failed ? INTERRUPTED : FINISHED;
    else if (link->jogging) *state = BUSY;
    else *state = link->session.phase == BAIZE_READY && now - link->last_wheel_ms < 250 ? READY : SYNCING;
    return count;
}

void panel_close(PanelLink *link)
{
    serial_write_frame(link->port, &(BaizeFrame){.type = BAIZE_STOP});
    serial_close(link->port);
    free(link);
}
