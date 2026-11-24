/* Public C interface with a deterministic transport and the real v1 decoder. */
#include <baize/chassis.h>
#include "serial_port.h"

#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <string.h>

static uint64_t now_ms;
static uint32_t board_ms;
static uint8_t input[4096];
static size_t pending;
static uint8_t delayed_tail[3];
static bool deliver_tail;
static bool acknowledge = true, disconnected, fault;
static unsigned stops, enables, targets;
static int32_t last_target[4];

static void enqueue(BaizeFrame frame)
{
    uint8_t wire[BAIZE_WIRE_MAX];
    size_t size = baize_encode(&frame, wire);
    assert(size && pending + size <= sizeof input);
    memcpy(input + pending, wire, size);
    pending += size;
}

static void wheel_sample(uint8_t state, uint8_t reason)
{
    enqueue((BaizeFrame){.type = BAIZE_STATUS,
        .status = {.sample_ms = board_ms, .drive_state = state, .stop_reason = reason, .supply_mv = UINT16_MAX,
                   .angle_mrad = {-1234, 1000, 2000, 0}, .velocity_mrad_s = {500, -500, -1000, 2000}}});
}

static void imu_sample(uint32_t tick)
{
    enqueue((BaizeFrame){.type = BAIZE_IMU,
        .imu = {.sample_ms = tick, .acceleration_mm_s2 = {0, -1000, 9807}, .gyro_mrad_s = {200, 0, -500}}});
}

uint64_t monotonic_ms(void) { return now_ms; }
SerialPort serial_open(const char *name)
{
    pending = 0;
    return strcmp(name, "board") == 0 ? (SerialPort)(uintptr_t)1 : SERIAL_INVALID;
}
void serial_close(SerialPort port) { (void)port; }
int serial_available(SerialPort port) { (void)port; return disconnected ? -1 : (int)pending; }

int serial_read(SerialPort port, uint8_t *data, size_t size)
{
    (void)port;
    if (disconnected) return -1;
    if (!pending) now_ms += 20;
    if (size > pending) size = pending;
    memcpy(data, input, size);
    memmove(input, input + size, pending - size);
    pending -= size;
    return (int)size;
}

bool serial_write_frame(SerialPort port, const BaizeFrame *frame)
{
    (void)port;
    if (disconnected) return false;
    switch (frame->type) {
    case BAIZE_STOP:
        ++stops;
        if (deliver_tail) {
            memcpy(input + pending, delayed_tail, sizeof delayed_tail);
            pending += sizeof delayed_tail;
            deliver_tail = false;
        }
        board_ms += 20;
        if (acknowledge) wheel_sample(fault ? BAIZE_FAULT : BAIZE_DISABLED,
                                      fault ? BAIZE_DRIVE_FAULT : BAIZE_REQUESTED_STOP);
        break;
    case BAIZE_ENABLE:
        ++enables;
        board_ms += 20;
        if (acknowledge) wheel_sample(BAIZE_ENABLED, BAIZE_RUNNING);
        break;
    case BAIZE_TARGET:
        ++targets;
        memcpy(last_target, frame->target_mrad_s, sizeof last_target);
        break;
    default:
        assert(false);
    }
    return true;
}

static void kinematics(void)
{
    baize_geometry geometry = baize_nominal_geometry();
    baize_wheel_speeds forward = baize_to_wheel_speeds(geometry, geometry.wheel_radius_m, 0);
    baize_wheel_speeds left = baize_to_wheel_speeds(geometry, 0, 2*geometry.wheel_radius_m/geometry.track_width_m);
    for (size_t i = 0; i < 4; ++i) {
        assert(fabs(forward.rad_s[i] - 1) < 1e-12);
        assert(fabs(left.rad_s[i] - (i % 2 ? 1 : -1)) < 1e-12);
    }
    geometry.wheel_radius_m = 0;
    assert(isnan(baize_to_wheel_speeds(geometry, 1, 0).rad_s[0]));
}

static void replayed_feedback(void)
{
    baize_chassis_state state;
    const baize_wheel_speeds stopped = {{0}};
    board_ms = 47501;
    baize_chassis *chassis = baize_chassis_open("board");
    assert(chassis && baize_chassis_enable(chassis));
    acknowledge = false;
    for (unsigned i = 0; i < 8; ++i) {
        board_ms = i % 2 ? 47561 : 47541;
        wheel_sample(BAIZE_ENABLED, BAIZE_RUNNING);
        imu_sample(i % 2 ? 47544 : 47524);
        assert(baize_chassis_read(chassis, &state));
        if (i == 1) assert(state.drive.valid && state.imu.valid);
        if (i >= 4) assert(!state.drive.valid && !state.imu.valid);
        now_ms += 100;
    }
    assert(!baize_chassis_write(chassis, &stopped));
    board_ms = 47581;
    wheel_sample(BAIZE_DISABLED, BAIZE_REQUESTED_STOP);
    imu_sample(47564);
    assert(baize_chassis_read(chassis, &state) && state.drive.valid && state.imu.valid);
    assert(!baize_chassis_write(chassis, &stopped));
    baize_chassis_close(chassis);
    acknowledge = true;
}

int main(void)
{
    kinematics();
    baize_chassis_state state;
    baize_wheel_speeds wheels = {{0.0625, -0.1875, 2, -2}};
    assert(!baize_chassis_open(NULL) && !baize_chassis_open("absent"));
    baize_chassis *chassis = baize_chassis_open("board");
    assert(chassis && stops == 1 && !baize_chassis_write(chassis, &wheels));
    assert(baize_chassis_read(chassis, &state) && state.drive.valid && !state.imu.valid);
    assert(isnan(state.drive.supply_v) && state.drive.angle_rad[0] == -1.234);
    assert(baize_chassis_enable(chassis) && enables == 1);

    /* Consume a backlog larger than one transport read and preserve independent timestamps. */
    for (unsigned i = 0; i < 12; ++i) imu_sample(i);
    board_ms += 20;
    wheel_sample(BAIZE_ENABLED, BAIZE_RUNNING);
    assert(baize_chassis_read(chassis, &state) && state.imu.valid && state.imu.sample_ms == 11);
    assert(state.imu.acceleration_m_s2[2] == 9.807 && state.imu.angular_velocity_rad_s[2] == -0.5);
    assert(state.drive.sample_ms == board_ms && state.drive.velocity_rad_s[3] == 2);
    assert(baize_chassis_write(chassis, &wheels) && targets == 1);
    assert(last_target[0] == 63 && last_target[1] == -188 && last_target[2] == 2000 && last_target[3] == -2000);
    wheels.rad_s[3] = NAN;
    assert(!baize_chassis_write(chassis, &wheels));
    wheels.rad_s[3] = 2.0001;
    assert(!baize_chassis_write(chassis, &wheels) && targets == 1);
    wheels = (baize_wheel_speeds){{0}};
    assert(baize_chassis_write(chassis, &wheels) && targets == 2 && last_target[0] == 0);

    now_ms += 200;
    board_ms += 200;
    wheel_sample(BAIZE_ENABLED, BAIZE_RUNNING);
    assert(baize_chassis_read(chassis, &state));
    now_ms += 50;
    imu_sample(11); /* Duplicate cannot keep the IMU valid. */
    assert(baize_chassis_read(chassis, &state) && !state.imu.valid && state.drive.valid);
    imu_sample(12);
    assert(baize_chassis_read(chassis, &state) && state.imu.valid);

    /* Only IMU is live. A buffered, late wheel sample must not revive motion. */
    acknowledge = false;
    now_ms += 250;
    imu_sample(13);
    assert(baize_chassis_read(chassis, &state) && state.imu.valid && !state.drive.valid);
    board_ms += 20;
    wheel_sample(BAIZE_ENABLED, BAIZE_RUNNING);
    assert(baize_chassis_read(chassis, &state) && !baize_chassis_write(chassis, &wheels));
    assert(targets == 2);
    acknowledge = true;
    assert(baize_chassis_disable(chassis) && baize_chassis_enable(chassis));
    board_ms = 0;
    wheel_sample(BAIZE_DISABLED, BAIZE_RESET);
    assert(baize_chassis_read(chassis, &state) && !state.imu.valid);
    assert(!baize_chassis_write(chassis, &wheels) && targets == 2);
    assert(baize_chassis_disable(chassis));
    disconnected = true;
    assert(!baize_chassis_read(chassis, &state) && !state.drive.valid && !state.imu.valid);
    disconnected = false;
    assert(!baize_chassis_read(chassis, &state)); /* A failed handle cannot resume. */
    baize_chassis_close(chassis);

    chassis = baize_chassis_open("board");
    assert(chassis && !baize_chassis_write(chassis, &wheels) && baize_chassis_enable(chassis));
    /* A buffered STOP status predating this request cannot acknowledge it. */
    acknowledge = false;
    board_ms += 20;
    wheel_sample(BAIZE_DISABLED, BAIZE_REQUESTED_STOP);
    assert(!baize_chassis_disable(chassis));
    /* A frame started before STOP may finish after the request was sent. */
    board_ms += 20;
    wheel_sample(BAIZE_DISABLED, BAIZE_REQUESTED_STOP);
    pending -= sizeof delayed_tail;
    memcpy(delayed_tail, input + pending, sizeof delayed_tail);
    assert(baize_chassis_read(chassis, &state));
    deliver_tail = true;
    assert(!baize_chassis_disable(chassis));
    acknowledge = true;
    assert(baize_chassis_disable(chassis));
    baize_chassis_close(chassis);
    fault = true;
    chassis = baize_chassis_open("board");
    assert(chassis && baize_chassis_read(chassis, &state) && state.drive.state == BAIZE_DRIVE_FAULTED);
    assert(!baize_chassis_enable(chassis) && !baize_chassis_write(chassis, &wheels));
    baize_chassis_close(chassis);
    acknowledge = false;
    assert(!baize_chassis_open("board"));
    fault = false;
    acknowledge = true;
    replayed_feedback();
    return 0;
}
