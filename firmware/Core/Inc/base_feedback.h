#ifndef BASE_FEEDBACK_H
#define BASE_FEEDBACK_H

#include "baize_encoders.h"

void base_feedback_start(void);
void base_feedback_status(const BaizeEncoders *encoders);
void base_feedback_poll(uint32_t now_ms);

#endif
