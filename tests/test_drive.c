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

static void faults_and_reset(void)
{
    BaizeDrive drive;
    baize_drive_init(&drive, 2000, 100);
    baize_drive_command(&drive, &enable, 0);
    baize_drive_command(&drive, &target, 1);
    baize_drive_fault(&drive, true);
    baize_drive_command(&drive, &stop, 2);
    baize_drive_command(&drive, &enable, 3);
    baize_drive_command(&drive, &target, 4);
    baize_drive_tick(&drive, 200);
    check_zero(&drive, BAIZE_FAULT, BAIZE_DRIVE_FAULT);
    baize_drive_fault(&drive, false);
    baize_drive_command(&drive, &target, 201);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_DRIVE_FAULT);
    baize_drive_command(&drive, &enable, 205);
    baize_drive_command(&drive, &target, 210);
    baize_drive_init(&drive, 2000, 100);
    check_zero(&drive, BAIZE_DISABLED, BAIZE_RESET);
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
    faults_and_reset();
    pi_feedback_and_stop_latch();
    wheel_feedforward();
    return 0;
}
