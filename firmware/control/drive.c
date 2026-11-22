#include "drive.h"

#include <math.h>
#include <string.h>

static void stop(Drive *drive, uint8_t state, uint8_t reason)
{
    memset(drive->target_mrad_s, 0, sizeof drive->target_mrad_s);
    drive->state = state;
    drive->stop_reason = reason;
    drive->reset_output = true;
}

void drive_init(Drive *drive, int32_t limit_mrad_s, uint32_t timeout_ms)
{
    *drive = (Drive){ .state = BAIZE_DISABLED, .stop_reason = BAIZE_RESET,
                          .limit_mrad_s = limit_mrad_s, .timeout_ms = timeout_ms, .reset_output = true };
}

void drive_tick(Drive *drive, uint32_t now_ms)
{
    if (drive->state == BAIZE_ENABLED && now_ms - drive->last_command_ms >= drive->timeout_ms) {
        stop(drive, BAIZE_DISABLED, BAIZE_TIMEOUT);
    }
}

void drive_command(Drive *drive, const BaizeFrame *frame, uint32_t now_ms)
{
    bool was_enabled = drive->state == BAIZE_ENABLED;
    drive_tick(drive, now_ms);
    switch (frame->type) {
    case BAIZE_STOP:
        if (drive->state != BAIZE_FAULT) stop(drive, BAIZE_DISABLED, BAIZE_REQUESTED_STOP);
        break;
    case BAIZE_ENABLE:
        if (!was_enabled && drive->state == BAIZE_DISABLED) {
            drive->state = BAIZE_ENABLED;
            drive->stop_reason = BAIZE_RUNNING;
            drive->last_command_ms = now_ms;
        }
        break;
    case BAIZE_TARGET:
        if (drive->state != BAIZE_ENABLED) return;
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            int32_t value = frame->target_mrad_s[i];
            if (value > drive->limit_mrad_s || value < -drive->limit_mrad_s) return;
        }
        memcpy(drive->target_mrad_s, frame->target_mrad_s, sizeof drive->target_mrad_s);
        drive->last_command_ms = now_ms;
        break;
    }
}

void drive_fault(Drive *drive)
{
    stop(drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
}

void drive_check_progress(Drive *drive, const volatile int64_t angle_mrad[BAIZE_WHEELS],
                          const int16_t applied_pwm[BAIZE_WHEELS], uint32_t now_ms)
{
    enum { MIN_PWM = 250, PROGRESS_MRAD = 2, STALL_MS = 350 };
    if (drive->reset_output) memset(drive->progress, 0, sizeof drive->progress);
    if (drive->state != BAIZE_ENABLED || drive->reset_output) return;

    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        WheelProgress *p = &drive->progress[i];
        int8_t direction = 0;
        if (drive->target_mrad_s[i] > 0 && applied_pwm[i] >= MIN_PWM) direction = 1;
        if (drive->target_mrad_s[i] < 0 && applied_pwm[i] <= -MIN_PWM) direction = -1;
        int64_t angle = angle_mrad[i];
        if (direction == 0 || direction != p->direction ||
            direction * (angle - p->mark_mrad) >= PROGRESS_MRAD) {
            p->direction = direction;
            p->mark_mrad = angle;
            p->since_ms = now_ms;
        } else if (now_ms - p->since_ms >= STALL_MS) {
            drive_fault(drive);
            return;
        }
    }
}

static float clamp_pwm(float value, int16_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return (float)-limit;
    return value;
}

int16_t wheel_pi_step(WheelPi *pi, int32_t target_mrad_s, int32_t measured_mrad_s,
                      uint32_t elapsed_ms, int16_t pwm_limit)
{
    if (target_mrad_s == 0) {
        pi->integral = 0;
        return 0;
    }
    float error = (float)target_mrad_s - (float)measured_mrad_s;
    float feedforward = pi->kv * (float)target_mrad_s + (target_mrad_s > 0 ? pi->ks : -pi->ks);
    float base = feedforward + pi->kp * error;
    float output = base + pi->integral;
    /* Integrate only while the error will not drive a saturated output further out. */
    if ((error > 0 && output < pwm_limit) || (error < 0 && output > -pwm_limit)) {
        float increment = pi->ki * error * ((float)elapsed_ms / 1000.0f);
        pi->integral = clamp_pwm(pi->integral + increment, pwm_limit);
    }
    return (int16_t)lroundf(clamp_pwm(base + pi->integral, pwm_limit));
}
