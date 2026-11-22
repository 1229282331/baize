#ifndef BAIZE_BASE_MOTION_H
#define BAIZE_BASE_MOTION_H

#include "encoders.h"

void base_motion_start(void);
void base_motion_command(const BaizeFrame *command, uint32_t received_ms);
void base_motion_update(const Encoders *encoders, uint32_t elapsed_ms,
                        uint8_t *drive_state, uint8_t *stop_reason);

#endif
