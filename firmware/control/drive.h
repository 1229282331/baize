#ifndef BAIZE_DRIVE_H
#define BAIZE_DRIVE_H

#include "baize_protocol.h"

typedef struct {
    int64_t mark_mrad;
    uint32_t since_ms;
    int8_t direction;
} WheelProgress;

typedef struct {
    uint8_t state;
    uint8_t stop_reason;
    int32_t target_mrad_s[BAIZE_WHEELS];
    int32_t limit_mrad_s;
    uint32_t timeout_ms;
    uint32_t last_command_ms;
    bool reset_output; /* Latched until the control loop applies a neutral cycle. */
    WheelProgress progress[BAIZE_WHEELS];
} Drive;

typedef struct {
    float kp, ki; /* PWM counts per velocity error and per integrated velocity error, respectively. */
    float integral;
    float ks; /* Friction compensation in PWM counts, applied in the target direction. */
    float kv; /* Velocity feedforward in PWM counts per mrad/s. */
} WheelPi;

/* Supply the calibrated nonnegative wheel speed limit and a positive command timeout. */
void drive_init(Drive *drive, int32_t limit_mrad_s, uint32_t timeout_ms);
void drive_command(Drive *drive, const BaizeFrame *frame, uint32_t now_ms);
void drive_tick(Drive *drive, uint32_t now_ms);
void drive_fault(Drive *drive); /* Latched until drive initialization at reset. */
/* Check the effort actually applied during the preceding encoder sample window. */
void drive_check_progress(Drive *drive, const volatile int64_t angle_mrad[BAIZE_WHEELS],
                          const int16_t applied_pwm[BAIZE_WHEELS], uint32_t now_ms);
int16_t wheel_pi_step(WheelPi *pi, int32_t target_mrad_s, int32_t measured_mrad_s,
                      uint32_t elapsed_ms, int16_t pwm_limit);

#endif
