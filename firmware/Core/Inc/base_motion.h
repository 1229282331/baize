#ifndef BAIZE_BASE_MOTION_H
#define BAIZE_BASE_MOTION_H

#include "baize_protocol.h"

void base_motion_start(void);
void base_motion_command(const BaizeFrame *command, uint32_t received_ms);
void base_motion_update(const int32_t velocity_mrad_s[BAIZE_WHEELS], uint32_t elapsed_ms,
                        uint8_t *drive_state, uint8_t *stop_reason);

#endif
