#include "baize_session.h"
#include "baize_drive.h"

#undef NDEBUG
#include <assert.h>
#include <string.h>

/* Synthetic board samples and a local clock. No serial port or motor is involved. */
static bool report(BaizeSession *host, const BaizeFrame *sample, uint32_t now)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(sample, wire);
    BaizeFrame received;
    for (size_t i = 0; i + 1 < length; ++i) {
        assert(!baize_session_receive(host, wire[i], now, &received));
    }
    if (!baize_session_receive(host, wire[length - 1], now, &received)) return false;
    assert(received.type == sample->type);
    return true;
}

static bool wheel_sample(BaizeSession *host, uint32_t board_ms, uint32_t host_ms, uint8_t state, uint8_t reason)
{
    return report(host, &(BaizeFrame){ .type = BAIZE_STATUS, .status = {
        .sample_ms = board_ms, .drive_state = state, .stop_reason = reason, .supply_mv = UINT16_MAX
    } }, host_ms);
}

static void status(BaizeSession *host, const BaizeDrive *drive, uint32_t now)
{
    assert(wheel_sample(host, now, now, drive->state, drive->stop_reason));
}

static void deliver(BaizeDrive *drive, const BaizeFrame *command, uint32_t now)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(command, wire);
    BaizeReceiver rx = {0};
    BaizeFrame received;
    for (size_t i = 0; i < length; ++i) {
        assert(baize_receive(&rx, BAIZE_TO_BASE, wire[i], &received) == (i + 1 == length));
    }
    baize_drive_command(drive, &received, now);
}

static void handshake(BaizeSession *host, BaizeDrive *drive, uint32_t now)
{
    BaizeFrame command;
    assert(!baize_session_enable(host, now, &command));
    assert(baize_session_next(host, now, &command) && command.type == BAIZE_STOP);
    deliver(drive, &command, now);
    status(host, drive, now);
    assert(baize_session_enable(host, now, &command) && command.type == BAIZE_ENABLE);
    deliver(drive, &command, now);
    status(host, drive, now + 1);
    assert(host->phase == BAIZE_ACTIVE && drive->state == BAIZE_ENABLED);
}

static void reconnect(void)
{
    BaizeDrive drive;
    BaizeSession host;
    BaizeFrame command, feedback;
    const int32_t old_target[] = {800, 400, 200, -300};
    const int32_t new_target[] = {10, 20, 30, 40};
    baize_drive_init(&drive, 2000, 100);
    baize_session_open(&host, 60);
    status(&host, &drive, 0); /* Feedback before the first STOP is ineligible. */
    assert(!baize_session_target(&host, old_target));
    handshake(&host, &drive, 1);
    assert(baize_session_target(&host, old_target));
    assert(baize_session_next(&host, 2, &command) && command.type == BAIZE_TARGET);
    deliver(&drive, &command, 2);
    assert(memcmp(drive.target_mrad_s, old_target, sizeof old_target) == 0);
    assert(baize_session_target(&host, old_target)); /* Pending when the port closes. */

    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(&(BaizeFrame){ .type = BAIZE_STATUS }, wire);
    for (size_t i = 0; i < length - 1; ++i) {
        assert(!baize_session_receive(&host, wire[i], 3, &feedback));
    }
    baize_session_open(&host, 60);
    assert(!baize_session_receive(&host, wire[length - 1], 4, &feedback));
    status(&host, &drive, 4);
    handshake(&host, &drive, 5);
    assert(!baize_session_next(&host, 6, &command));
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        assert(drive.target_mrad_s[i] == 0);
    }
    assert(baize_session_target(&host, new_target));
    assert(baize_session_next(&host, 7, &command) && command.type == BAIZE_TARGET);
    deliver(&drive, &command, 7);
    assert(memcmp(drive.target_mrad_s, new_target, sizeof new_target) == 0);
    assert(!baize_session_next(&host, 8, &command));
}

static void lost_feedback(void)
{
    BaizeDrive drive;
    BaizeSession host;
    BaizeFrame command, ignored;
    const int32_t target[] = {10, 20, 30, 40};
    baize_drive_init(&drive, 2000, 100);
    baize_session_open(&host, 60);
    handshake(&host, &drive, UINT32_MAX - 30);
    assert(baize_session_target(&host, target));
    assert(report(&host, &(BaizeFrame){ .type = BAIZE_IMU }, 29));
    assert(baize_session_next(&host, 30, &command) && command.type == BAIZE_STOP);
    assert(!baize_session_target(&host, target));
    status(&host, &drive, 31); /* Still enabled while STOP is in flight. */
    assert(!baize_session_enable(&host, 31, &ignored));
    deliver(&drive, &command, 32);
    status(&host, &drive, 33);
    assert(host.phase == BAIZE_READY);
    assert(baize_session_enable(&host, 34, &command));
    deliver(&drive, &command, 34);
    status(&host, &drive, 35);
    assert(baize_session_target(&host, target));
    status(&host, &drive, 100); /* A late status arrives before the sender polls. */
    assert(baize_session_next(&host, 101, &command) && command.type == BAIZE_STOP);
    baize_drive_fault(&drive, true);
    status(&host, &drive, 102);
    assert(!baize_session_enable(&host, 103, &command));
    assert(baize_session_next(&host, 103, &command) && command.type == BAIZE_STOP);
}

static void lost_enable_and_stop(void)
{
    BaizeDrive drive;
    BaizeSession host;
    BaizeFrame command;
    const int32_t target[] = {10, 20, 30, 40};
    baize_drive_init(&drive, 2000, 100);
    baize_session_open(&host, 60);
    assert(baize_session_next(&host, 0, &command) && command.type == BAIZE_STOP);
    deliver(&drive, &command, 0);
    status(&host, &drive, 1);
    assert(baize_session_enable(&host, 2, &command)); /* Simulate a lost ENABLE. */
    status(&host, &drive, 60);
    assert(!baize_session_target(&host, target));
    assert(baize_session_next(&host, 62, &command) && command.type == BAIZE_STOP);
    handshake(&host, &drive, 63);
    assert(baize_session_target(&host, target));
    baize_session_stop(&host);
    assert(baize_session_next(&host, 64, &command) && command.type == BAIZE_STOP);
    deliver(&drive, &command, 64);
    status(&host, &drive, 65);
    assert(!baize_session_next(&host, 66, &command));
    assert(!baize_session_enable(&host, 125, &command)); /* Idle feedback has expired. */
}

static void stop_ack_and_restart(void)
{
    BaizeSession host;
    BaizeFrame command;
    const int32_t target[] = {500, 0, 0, 0};
    baize_session_open(&host, 60);
    assert(baize_session_next(&host, 0, &command) && command.type == BAIZE_STOP);
    assert(wheel_sample(&host, 1000, 1, BAIZE_DISABLED, BAIZE_RESET));
    assert(!baize_session_enable(&host, 2, &command));
    assert(wheel_sample(&host, 1020, 3, BAIZE_DISABLED, BAIZE_TIMEOUT));
    assert(!baize_session_enable(&host, 3, &command));
    assert(baize_session_next(&host, 4, &command) && command.type == BAIZE_STOP);
    assert(wheel_sample(&host, 1040, 5, BAIZE_DISABLED, BAIZE_REQUESTED_STOP));
    assert(baize_session_enable(&host, 6, &command));
    assert(wheel_sample(&host, 1060, 7, BAIZE_ENABLED, BAIZE_RUNNING));
    assert(baize_session_target(&host, target));

    /* No boot frame was delivered. A younger board clock still cancels the queued target. */
    assert(wheel_sample(&host, 20, 8, BAIZE_ENABLED, BAIZE_RUNNING));
    assert(baize_session_next(&host, 9, &command) && command.type == BAIZE_STOP);
    assert(!baize_session_target(&host, target));
    assert(wheel_sample(&host, 40, 10, BAIZE_DISABLED, BAIZE_REQUESTED_STOP));
    assert(baize_session_enable(&host, 11, &command));
    assert(wheel_sample(&host, 60, 12, BAIZE_ENABLED, BAIZE_RUNNING));
    assert(!baize_session_next(&host, 13, &command));

    /* A reboot or fault can land on exactly the preceding sample timestamp. */
    assert(baize_session_target(&host, target));
    assert(wheel_sample(&host, 60, 14, BAIZE_DISABLED, BAIZE_RESET));
    assert(baize_session_next(&host, 15, &command) && command.type == BAIZE_STOP);
    assert(wheel_sample(&host, 80, 16, BAIZE_DISABLED, BAIZE_REQUESTED_STOP));
    assert(baize_session_enable(&host, 17, &command));
    assert(wheel_sample(&host, 100, 18, BAIZE_DISABLED, BAIZE_RESET));
    assert(baize_session_next(&host, 19, &command) && command.type == BAIZE_STOP);
}

static void sample_freshness(void)
{
    BaizeSession host;
    BaizeFrame command;
    BaizeFrame imu = { .type = BAIZE_IMU, .imu.sample_ms = 0 };
    const int32_t target[] = {500, 0, 0, 0};
    baize_session_open(&host, 60);
    assert(baize_session_next(&host, 0, &command) && command.type == BAIZE_STOP);
    assert(wheel_sample(&host, UINT32_MAX - 19, 1, BAIZE_DISABLED, BAIZE_REQUESTED_STOP));
    assert(baize_session_enable(&host, 2, &command));
    assert(wheel_sample(&host, 0, 3, BAIZE_ENABLED, BAIZE_RUNNING));
    assert(baize_session_target(&host, target));
    assert(baize_session_next(&host, 4, &command) && command.type == BAIZE_TARGET);
    assert(baize_session_target(&host, target));
    assert(report(&host, &imu, 5));
    assert(!report(&host, &imu, 25));
    assert(!wheel_sample(&host, 0, 42, BAIZE_ENABLED, BAIZE_RUNNING));
    imu.imu.sample_ms = 20;
    assert(report(&host, &imu, 60));
    assert(baize_session_next(&host, 63, &command) && command.type == BAIZE_STOP);
    assert(!baize_session_target(&host, target));
}

int main(void)
{
    reconnect();
    lost_feedback();
    lost_enable_and_stop();
    stop_ack_and_restart();
    sample_freshness();
    return 0;
}
