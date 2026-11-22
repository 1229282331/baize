#include "baize_drive.h"

#undef NDEBUG
#include <assert.h>
#include <string.h>

/* Bench-independent test inputs. */
static const BaizeFrame enable = { .type = BAIZE_ENABLE };
static const BaizeFrame stop = { .type = BAIZE_STOP };
static const BaizeFrame target = { .type = BAIZE_TARGET, .target_mrad_s = {2000, -2000, 750, -500} };

static void check_zero(const BaizeDrive *drive, uint8_t state, uint8_t reason)
{
    const int32_t zero[BAIZE_WHEELS] = {0};
    assert(drive->state == state && drive->stop_reason == reason);
    assert(memcmp(drive->target_mrad_s, zero, sizeof zero) == 0);
}

static void enable_and_expire(void)
{
    BaizeDrive drive;
    baize_drive_init(&drive, 2000, 100);
    baize_drive_command(&drive, &target, 7);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_RESET);
    baize_drive_command(&drive, &enable, 10);
    baize_drive_command(&drive, &enable, 109);
    check_zero(&drive, BAIZE_ENABLED, BAIZE_RUNNING);
    baize_drive_tick(&drive, 110);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_TIMEOUT);

    baize_drive_command(&drive, &enable, 120);
    baize_drive_command(&drive, &target, 140);
    baize_drive_command(&drive, &enable, 170);
    BaizeFrame invalid = target;
    invalid.target_mrad_s[0] = 100;
    invalid.target_mrad_s[3] = 2001;
    baize_drive_command(&drive, &invalid, 220);
    invalid.target_mrad_s[3] = INT32_MIN;
    baize_drive_command(&drive, &invalid, 239);
    baize_drive_tick(&drive, 239);
    assert(drive.state == BAIZE_ENABLED);
    assert(memcmp(drive.target_mrad_s, target.target_mrad_s, sizeof drive.target_mrad_s) == 0);
    /* The first event at the deadline can be a new command, before the next control tick. */
    baize_drive_command(&drive, &target, 240);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_TIMEOUT);
    baize_drive_command(&drive, &enable, 245);
    check_zero(&drive, BAIZE_ENABLED, BAIZE_RUNNING);
    baize_drive_command(&drive, &target, 246);
    baize_drive_command(&drive, &stop, 247);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_REQUESTED_STOP);
}

static void crc_and_clock_wrap(void)
{
    BaizeDrive drive;
    baize_drive_init(&drive, 2000, 100);
    baize_drive_command(&drive, &enable, UINT32_MAX - 80);
    baize_drive_command(&drive, &target, UINT32_MAX - 49);
    BaizeReceiver rx = {0};
    BaizeFrame incoming;
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(&target, wire);
    wire[3] ^= 1;
    for (size_t i = 0; i < length; ++i) {
        if (baize_receive(&rx, BAIZE_TO_BASE, wire[i], &incoming)) {
            baize_drive_command(&drive, &incoming, 49);
        }
    }
    baize_drive_tick(&drive, 49);
    assert(drive.state == BAIZE_ENABLED);
    baize_drive_tick(&drive, 50);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_TIMEOUT);
}

static void wheel_progress_and_latched_fault(void)
{
    BaizeDrive drive;
    int64_t angle[BAIZE_WHEELS] = {0};
    int16_t pwm[BAIZE_WHEELS] = {249, -250, 0, -700};
    uint32_t start = UINT32_MAX - 100;
    baize_drive_init(&drive, 2000, 100);
    baize_drive_command(&drive, &enable, start);
    drive.reset_output = false;
    for (unsigned dt = 0; dt <= 355; ++dt) {
        if (dt % 20 == 0) baize_drive_command(&drive, &target, start + dt);
        angle[1] = -(int64_t)dt; /* Honest progress on an independent wheel. */
        angle[3] = dt <= 5 ? 0 : -2 + (int64_t)(dt % 2);
        baize_drive_check_progress(&drive, angle, pwm, start + dt);
        assert(drive.state == BAIZE_ENABLED);
    }
    baize_drive_check_progress(&drive, angle, pwm, start + 356);
    check_zero(&drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
    assert(drive.reset_output);
    baize_drive_command(&drive, &stop, start + 357);
    baize_drive_command(&drive, &enable, start + 358);
    baize_drive_command(&drive, &target, start + 359);
    baize_drive_tick(&drive, start + 1000);
    check_zero(&drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
    baize_drive_init(&drive, 2000, 100);
    baize_drive_command(&drive, &target, 0);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_RESET);
}

static void progress_after_stop_and_reversal(void)
{
    BaizeDrive drive;
    int64_t angle[BAIZE_WHEELS] = {0};
    int16_t pwm[BAIZE_WHEELS] = {250, 0, 0, 0};
    BaizeFrame demand = target;
    baize_drive_init(&drive, 2000, 1000);
    baize_drive_command(&drive, &enable, 0);
    baize_drive_command(&drive, &demand, 1);
    baize_drive_check_progress(&drive, angle, pwm, 1); /* Startup neutral. */
    drive.reset_output = false;
    baize_drive_check_progress(&drive, angle, pwm, 2);
    baize_drive_command(&drive, &stop, 348);
    baize_drive_command(&drive, &enable, 349);
    baize_drive_command(&drive, &demand, 350);
    baize_drive_check_progress(&drive, angle, pwm, 351);
    drive.reset_output = false;
    baize_drive_check_progress(&drive, angle, pwm, 352);
    demand.target_mrad_s[0] = -500;
    baize_drive_command(&drive, &demand, 650);
    baize_drive_check_progress(&drive, angle, pwm, 651); /* Braking the old direction. */
    pwm[0] = -250;
    baize_drive_check_progress(&drive, angle, pwm, 652);
    angle[0] = 50; /* Movement in the wrong direction cannot keep the drive alive. */
    baize_drive_check_progress(&drive, angle, pwm, 1001);
    assert(drive.state == BAIZE_ENABLED);
    baize_drive_check_progress(&drive, angle, pwm, 1002);
    check_zero(&drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
}

static void pi_feedback_and_stop_latch(void)
{
    BaizeWheelPi pi = {.kp = 0.5f, .ki = 2.0f};
    assert(baize_wheel_effort(&pi, 100, 50, 10, 100) == 26);
    assert(baize_wheel_effort(&pi, 100, 50, 20, 100) == 28);
    for (int i = 0; i < 200; ++i) assert(baize_wheel_effort(&pi, 1000, 0, 5, 100) == 100);
    assert(pi.integral == 3.0f);
    assert(baize_wheel_effort(&pi, -500, 0, 5, 100) == -100);
    assert(baize_wheel_effort(&pi, 0, 500, 5, 100) == 0 && pi.integral == 0);

    BaizeDrive drive;
    baize_drive_init(&drive, 2000, 150);
    drive.reset_output = false;
    baize_drive_command(&drive, &enable, 20);
    baize_drive_command(&drive, &target, 21);
    baize_drive_command(&drive, &stop, 22);
    baize_drive_command(&drive, &enable, 23);
    baize_drive_command(&drive, &target, 24);
    assert(drive.state == BAIZE_ENABLED && drive.reset_output);
}

static void wheel_feedforward(void)
{
    BaizeWheelPi pi = {.kp = 0.25f, .ki = 2.0f, .ks = 20.0f, .kv = 0.5f};
    /* At target speed the two feedforward terms supply the running effort. */
    assert(baize_wheel_effort(&pi, 100, 100, 10, 100) == 70);
    assert(baize_wheel_effort(&pi, -100, -100, 10, 100) == -70);
    assert(baize_wheel_effort(&pi, 100, 0, 10, 100) == 97);
    /* Include feedforward in the saturation check and allow I to unwind. */
    assert(baize_wheel_effort(&pi, 200, 190, 10, 100) == 100);
    assert(pi.integral == 2.0f);
    assert(baize_wheel_effort(&pi, 200, 210, 100, 100) == 100);
    assert(pi.integral == 0.0f);
    assert(baize_wheel_effort(&pi, -100, 0, 10, 100) == -97);
    assert(baize_wheel_effort(&pi, 0, -100, 10, 100) == 0 && pi.integral == 0.0f);
}

int main(void)
{
    enable_and_expire();
    crc_and_clock_wrap();
    wheel_progress_and_latched_fault();
    progress_after_stop_and_reversal();
    pi_feedback_and_stop_latch();
    wheel_feedforward();
    return 0;
}
