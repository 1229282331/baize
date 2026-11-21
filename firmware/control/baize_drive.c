#include "baize_drive.h"

#include <math.h>
#include <string.h>

static void stop(BaizeDrive *drive, uint8_t state, uint8_t reason)
{
    memset(drive->target_mrad_s, 0, sizeof drive->target_mrad_s);
    drive->state = state;
    drive->stop_reason = reason;
    drive->reset_output = true;
}

void baize_drive_init(BaizeDrive *drive, int32_t limit_mrad_s, uint32_t timeout_ms)
{
    *drive = (BaizeDrive){ .state = BAIZE_DISABLED, .stop_reason = BAIZE_RESET,
                          .limit_mrad_s = limit_mrad_s, .timeout_ms = timeout_ms, .reset_output = true };
}

void baize_drive_tick(BaizeDrive *drive, uint32_t now_ms)
{
    if (drive->state == BAIZE_ENABLED && now_ms - drive->last_command_ms >= drive->timeout_ms) {
        stop(drive, BAIZE_DISABLED, BAIZE_TIMEOUT);
    }
}

void baize_drive_command(BaizeDrive *drive, const BaizeFrame *frame, uint32_t now_ms)
{
    bool was_enabled = drive->state == BAIZE_ENABLED;
    baize_drive_tick(drive, now_ms);
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

void baize_drive_fault(BaizeDrive *drive, bool active)
{
    if (active) {
        stop(drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
    } else if (drive->state == BAIZE_FAULT) {
        drive->state = BAIZE_DISABLED;
    }
}

static float clamp_pwm(float value, int16_t limit)
{
    if (value > limit) return limit;
    if (value < -limit) return (float)-limit;
    return value;
}

int16_t baize_wheel_effort(BaizeWheelPi *pi, int32_t target_mrad_s, int32_t measured_mrad_s,
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
