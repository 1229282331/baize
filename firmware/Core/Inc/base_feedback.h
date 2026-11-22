#ifndef BASE_FEEDBACK_H
#define BASE_FEEDBACK_H

#include "encoders.h"

void base_feedback_start(void);
void base_feedback_queue_status(const Encoders *encoders, uint8_t state, uint8_t reason);
void base_feedback_poll(uint32_t now_ms);

#endif
