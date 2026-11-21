#include "baize_drive.h"

#include <string.h>

static void stop(BaizeDrive *drive, uint8_t state, uint8_t reason)
{
    memset(drive->target_mrad_s, 0, sizeof drive->target_mrad_s);
    drive->state = state;
    drive->stop_reason = reason;
}

void baize_drive_init(BaizeDrive *drive, int32_t limit_mrad_s, uint32_t timeout_ms)
{
    *drive = (BaizeDrive){ .state = BAIZE_DISABLED, .stop_reason = BAIZE_RESET,
                          .limit_mrad_s = limit_mrad_s, .timeout_ms = timeout_ms };
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
