#ifndef CHASSIS_IO_H
#define CHASSIS_IO_H

#include <baize/chassis.h>
#include "baize_session.h"
#include "serial_port.h"

/* The recorders consume every sample, including those received during handshakes. */
typedef struct {
    uint8_t kind;
    union {
        baize_drive_sample drive;
        baize_imu_sample imu;
    };
} FeedbackSample;

/* A 256-byte read can finish at most 8 frames, including a partial first frame. */
enum { FEEDBACK_BATCH = 8 };

baize_chassis *chassis_connect(const char *port);
int chassis_poll(baize_chassis *chassis, FeedbackSample samples[FEEDBACK_BATCH]);
BaizeHostPhase chassis_phase(const baize_chassis *chassis);
int chassis_request_enable(baize_chassis *chassis); /* 1 sent, 0 refused, -1 I/O error. */
bool chassis_request_stop(baize_chassis *chassis);
#endif
