#ifndef BAIZE_DRIVE_H
#define BAIZE_DRIVE_H

#include "baize_protocol.h"

typedef struct {
    uint8_t state;
    uint8_t stop_reason;
    int32_t target_mrad_s[BAIZE_WHEELS];
    int32_t limit_mrad_s;
    uint32_t timeout_ms;
    uint32_t last_command_ms;
    bool reset_output; /* Latched until the control loop applies a neutral cycle. */
} BaizeDrive;

typedef struct {
    float kp, ki; /* PWM counts per velocity error and per integrated velocity error, respectively. */
    float integral;
} BaizeWheelPi;

/* Supply the calibrated nonnegative wheel speed limit and a positive command timeout. */
void baize_drive_init(BaizeDrive *drive, int32_t limit_mrad_s, uint32_t timeout_ms);
void baize_drive_command(BaizeDrive *drive, const BaizeFrame *frame, uint32_t now_ms);
void baize_drive_tick(BaizeDrive *drive, uint32_t now_ms);
void baize_drive_fault(BaizeDrive *drive, bool active);
int16_t baize_wheel_effort(BaizeWheelPi *pi, int32_t target_mrad_s, int32_t measured_mrad_s,
                          uint32_t elapsed_ms, int16_t pwm_limit);

#endif
