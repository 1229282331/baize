#include "baize_session.h"
#include "baize_drive.h"

#undef NDEBUG
#include <assert.h>
#include <string.h>

/* Synthetic board samples and a local clock. No serial port or motor is involved. */
static void report(BaizeSession *host, const BaizeFrame *sample, uint32_t now)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t length = baize_encode(sample, wire);
    BaizeFrame received;
    for (size_t i = 0; i < length; ++i) {
        assert(baize_session_receive(host, wire[i], now, &received) == (i + 1 == length));
    }
    assert(received.type == sample->type);
}

static void status(BaizeSession *host, const BaizeDrive *drive, uint32_t now)
{
    report(host, &(BaizeFrame){ .type = BAIZE_STATUS, .status = {
        .sample_ms = now, .drive_state = drive->state, .stop_reason = drive->stop_reason,
        .supply_mv = UINT16_MAX
    } }, now);
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
    status(host, drive, now);
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
    handshake(&host, &drive, UINT32_MAX - 29);
    assert(baize_session_target(&host, target));
    report(&host, &(BaizeFrame){ .type = BAIZE_IMU }, 29);
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

int main(void)
{
    reconnect();
    lost_feedback();
    lost_enable_and_stop();
    return 0;
}
