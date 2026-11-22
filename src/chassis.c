#include "chassis_io.h"

#include <math.h>
#include <stdlib.h>

enum { FEEDBACK_TIMEOUT_MS = 250, STOP_RETRY_MS = 20, HANDSHAKE_TIMEOUT_MS = 1000 };

struct baize_chassis {
    SerialPort port;
    BaizeSession drive;
    baize_chassis_state state;
    uint64_t last_wheel_rx_ms, last_imu_rx_ms, next_stop_ms;
    int bytes_before_request;
    bool frame_before_request;
    bool io_failed;
};

baize_geometry baize_nominal_geometry(void)
{
    return (baize_geometry){.wheel_radius_m = 0.496371 / (2.0 * 3.14159265358979323846),
                            .track_width_m = 0.265};
}

baize_wheel_speeds baize_to_wheel_speeds(baize_geometry geometry, double forward_m_s, double yaw_rate_rad_s)
{
    double left = NAN, right = NAN;
    if (geometry.wheel_radius_m > 0 && isfinite(geometry.wheel_radius_m) &&
        geometry.track_width_m > 0 && isfinite(geometry.track_width_m) &&
        isfinite(forward_m_s) && isfinite(yaw_rate_rad_s)) {
        double turn_m_s = yaw_rate_rad_s * (geometry.track_width_m / 2);
        left = (forward_m_s - turn_m_s) / geometry.wheel_radius_m;
        right = (forward_m_s + turn_m_s) / geometry.wheel_radius_m;
    }
    return (baize_wheel_speeds){{left, right, left, right}};
}

static bool io_error(baize_chassis *chassis)
{
    if (!chassis->io_failed) serial_write_frame(chassis->port, &(BaizeFrame){.type = BAIZE_STOP});
    chassis->io_failed = true;
    baize_session_stop(&chassis->drive);
    chassis->state.drive.valid = false;
    chassis->state.imu.valid = false;
    return false;
}

static bool send_frame(baize_chassis *chassis, const BaizeFrame *frame)
{
    if (!serial_write_frame(chassis->port, frame)) return io_error(chassis);
    return true;
}

static bool send_request(baize_chassis *chassis, const BaizeFrame *frame)
{
    /* Recorders still receive queued samples, but only a later frame can confirm this request. */
    chassis->bytes_before_request = serial_available(chassis->port);
    if (chassis->bytes_before_request < 0) return io_error(chassis);
    chassis->frame_before_request = chassis->drive.rx.length != 0 || chassis->drive.rx.discarding;
    return send_frame(chassis, frame);
}

/* New targets are consumed only by write(). Calls from the read path service STOP. */
static bool send_next(baize_chassis *chassis, uint64_t now_ms)
{
    if (chassis->io_failed) return false;
    if (chassis->drive.phase >= BAIZE_READY && now_ms - chassis->last_wheel_rx_ms >= FEEDBACK_TIMEOUT_MS)
        baize_session_stop(&chassis->drive);
    if (chassis->drive.phase == BAIZE_WAIT_DISABLED && now_ms < chassis->next_stop_ms) return true;
    bool retrying_stop = chassis->drive.phase == BAIZE_WAIT_DISABLED;
    BaizeFrame command;
    if (!baize_session_next(&chassis->drive, (uint32_t)now_ms, &command)) return true;
    if (command.type == BAIZE_STOP) {
        chassis->next_stop_ms = now_ms + STOP_RETRY_MS;
        if (!retrying_stop) return send_request(chassis, &command);
    }
    return send_frame(chassis, &command);
}

baize_chassis *chassis_connect(const char *port)
{
    if (!port || !*port) return NULL;
    baize_chassis *chassis = calloc(1, sizeof *chassis);
    if (!chassis) return NULL;
    chassis->port = serial_open(port);
    if (chassis->port == SERIAL_INVALID) {
        free(chassis);
        return NULL;
    }
    baize_session_open(&chassis->drive, FEEDBACK_TIMEOUT_MS);
    chassis->state.drive.supply_v = NAN;
    if (!send_next(chassis, monotonic_ms())) {
        baize_chassis_close(chassis);
        return NULL;
    }
    return chassis;
}

static void store_sample(baize_chassis *chassis, const BaizeFrame *frame, uint64_t now_ms,
                         FeedbackSample *sample)
{
    if (sample) sample->kind = frame->type;
    if (frame->type == BAIZE_IMU) {
        baize_imu_sample *imu = &chassis->state.imu;
        imu->sample_ms = frame->imu.sample_ms;
        imu->valid = true;
        for (size_t i = 0; i < 3; ++i) {
            imu->acceleration_m_s2[i] = frame->imu.acceleration_mm_s2[i] / 1000.0;
            imu->angular_velocity_rad_s[i] = frame->imu.gyro_mrad_s[i] / 1000.0;
        }
        chassis->last_imu_rx_ms = now_ms;
        if (sample) sample->imu = *imu;
    } else {
        const BaizeStatus *raw = &frame->status;
        baize_drive_sample *drive = &chassis->state.drive;
        drive->sample_ms = raw->sample_ms;
        drive->valid = true;
        drive->state = raw->drive_state;
        drive->stop_reason = raw->stop_reason;
        drive->supply_v = raw->supply_mv == UINT16_MAX ? NAN : raw->supply_mv / 1000.0;
        for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) {
            drive->angle_rad[i] = (double)raw->angle_mrad[i] / 1000.0;
            drive->velocity_rad_s[i] = raw->velocity_mrad_s[i] / 1000.0;
        }
        chassis->last_wheel_rx_ms = now_ms;
        if (!chassis->drive.have_imu_sample) chassis->state.imu.valid = false;
        if (sample) sample->drive = *drive;
    }
}

/* Returns bytes consumed, so a partial frame cannot prematurely end a snapshot read. */
static int receive_chunk(baize_chassis *chassis, size_t size, FeedbackSample *samples, int *count)
{
    uint8_t data[256];
    if (size > sizeof data) size = sizeof data;
    int received = serial_read(chassis->port, data, size);
    if (received < 0) {
        io_error(chassis);
        return -1;
    }
    uint64_t now_ms = monotonic_ms();
    for (int i = 0; i < received; ++i) {
        bool prior = chassis->bytes_before_request > 0 || chassis->frame_before_request;
        if (chassis->bytes_before_request > 0) --chassis->bytes_before_request;
        chassis->frame_before_request = prior && data[i] != 0;
        BaizeHostPhase before = chassis->drive.phase;
        BaizeFrame frame;
        if (baize_session_receive(&chassis->drive, data[i], (uint32_t)now_ms, &frame)) {
            if (prior && ((before == BAIZE_WAIT_DISABLED && chassis->drive.phase == BAIZE_READY) ||
                          (before == BAIZE_WAIT_ENABLED && chassis->drive.phase == BAIZE_ACTIVE)))
                chassis->drive.phase = before;
            store_sample(chassis, &frame, now_ms, samples ? &samples[*count] : NULL);
            ++*count;
        }
    }
    return received;
}

int chassis_poll(baize_chassis *chassis, FeedbackSample samples[FEEDBACK_BATCH])
{
    int count = 0;
    if (!send_next(chassis, monotonic_ms()) || receive_chunk(chassis, 256, samples, &count) < 0 ||
        !send_next(chassis, monotonic_ms())) return -1;
    return count;
}

bool baize_chassis_read(baize_chassis *chassis, baize_chassis_state *state)
{
    if (!state) return false;
    *state = (baize_chassis_state){0};
    if (!chassis || !send_next(chassis, monotonic_ms())) return false;
    int bytes = serial_available(chassis->port);
    if (bytes < 0) return io_error(chassis);
    while (bytes > 0) {
        int count = 0;
        int received = receive_chunk(chassis, (size_t)bytes, NULL, &count);
        if (received < 0) return false;
        if (!received) break;
        bytes -= received;
    }
    uint64_t now_ms = monotonic_ms();
    if (!send_next(chassis, now_ms)) return false;
    *state = chassis->state;
    state->drive.valid &= now_ms - chassis->last_wheel_rx_ms < FEEDBACK_TIMEOUT_MS;
    state->imu.valid &= now_ms - chassis->last_imu_rx_ms < FEEDBACK_TIMEOUT_MS;
    return true;
}

BaizeHostPhase chassis_phase(const baize_chassis *chassis)
{
    return chassis->drive.phase;
}

int chassis_request_enable(baize_chassis *chassis)
{
    uint64_t now_ms = monotonic_ms();
    if (!send_next(chassis, now_ms)) return -1;
    BaizeFrame command;
    if (!baize_session_enable(&chassis->drive, (uint32_t)now_ms, &command)) return 0;
    return send_request(chassis, &command) ? 1 : -1;
}

bool chassis_request_stop(baize_chassis *chassis)
{
    baize_session_stop(&chassis->drive);
    return send_next(chassis, monotonic_ms());
}

static bool wait_for_phase(baize_chassis *chassis, BaizeHostPhase phase)
{
    uint64_t deadline = monotonic_ms() + HANDSHAKE_TIMEOUT_MS;
    while (chassis_phase(chassis) != phase) {
        if (chassis->state.drive.valid && chassis->state.drive.state == BAIZE_FAULT) return false;
        if (phase == BAIZE_ACTIVE && chassis_phase(chassis) != BAIZE_WAIT_ENABLED) return false;
        if (monotonic_ms() >= deadline || chassis_poll(chassis, NULL) < 0) return false;
    }
    return true;
}

baize_chassis *baize_chassis_open(const char *port)
{
    baize_chassis *chassis = chassis_connect(port);
    if (!chassis) return NULL;
    if (wait_for_phase(chassis, BAIZE_READY)) return chassis;
    if (!chassis->io_failed && chassis->state.drive.valid &&
        chassis->state.drive.state == BAIZE_FAULT) return chassis;
    baize_chassis_close(chassis);
    return NULL;
}

bool baize_chassis_enable(baize_chassis *chassis)
{
    baize_chassis_state state;
    if (!baize_chassis_read(chassis, &state) || !state.drive.valid ||
        chassis_request_enable(chassis) != 1) return false;
    if (wait_for_phase(chassis, BAIZE_ACTIVE)) return true;
    chassis_request_stop(chassis);
    return false;
}

bool baize_chassis_write(baize_chassis *chassis, const baize_wheel_speeds *wheels)
{
    if (!chassis || !wheels) return false;
    uint64_t now_ms = monotonic_ms();
    if (!send_next(chassis, now_ms) || chassis_phase(chassis) != BAIZE_ACTIVE) return false;
    int32_t target[BAIZE_WHEEL_COUNT];
    for (size_t i = 0; i < BAIZE_WHEEL_COUNT; ++i) {
        double speed = wheels->rad_s[i];
        if (!(speed >= -2.0 && speed <= 2.0)) return false;
        target[i] = (int32_t)lround(speed * 1000.0);
    }
    return baize_session_target(&chassis->drive, target) && send_next(chassis, now_ms);
}

bool baize_chassis_disable(baize_chassis *chassis)
{
    return chassis && chassis_request_stop(chassis) && wait_for_phase(chassis, BAIZE_READY);
}

void baize_chassis_close(baize_chassis *chassis)
{
    if (!chassis) return;
    serial_write_frame(chassis->port, &(BaizeFrame){.type = BAIZE_STOP});
    serial_close(chassis->port);
    free(chassis);
}
