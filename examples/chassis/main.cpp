#include "baize_session.h"
#include "serial_port.h"

#include <cstdio>

struct Connection {
    SerialPort port;
    BaizeSession session;
    uint64_t next_send_ms;
};

static bool exchange(Connection &link)
{
    uint64_t now = monotonic_ms();
    if (now >= link.next_send_ms || link.session.phase == BAIZE_SEND_STOP) {
        BaizeFrame command{};
        if (baize_session_next(&link.session, static_cast<uint32_t>(now), &command) &&
            !serial_write_frame(link.port, &command)) return false;
        link.next_send_ms = now + 20;
    }

    uint8_t bytes[256];
    int count = serial_read(link.port, bytes, sizeof bytes);
    if (count < 0) return false;
    uint32_t received_ms = static_cast<uint32_t>(monotonic_ms());
    for (int i = 0; i < count; ++i) {
        BaizeFrame frame{};
        if (!baize_session_receive(&link.session, bytes[i], received_ms, &frame)) continue;
        if (frame.type == BAIZE_STATUS) {
            std::printf("wheels %lu ms", static_cast<unsigned long>(frame.status.sample_ms));
            for (size_t j = 0; j < BAIZE_WHEELS; ++j)
                std::printf("  %.3f rad %.3f rad/s", frame.status.angle_mrad[j] / 1000.0,
                            frame.status.velocity_mrad_s[j] / 1000.0);
            std::puts("");
        } else if (frame.type == BAIZE_IMU) {
            const BaizeImu &imu = frame.imu;
            std::printf("imu %lu ms  %.3f %.3f %.3f m/s^2  %.3f %.3f %.3f rad/s\n",
                        static_cast<unsigned long>(imu.sample_ms),
                        imu.acceleration_mm_s2[0] / 1000.0, imu.acceleration_mm_s2[1] / 1000.0,
                        imu.acceleration_mm_s2[2] / 1000.0, imu.gyro_mrad_s[0] / 1000.0,
                        imu.gyro_mrad_s[1] / 1000.0, imu.gyro_mrad_s[2] / 1000.0);
        }
    }
    return true;
}

static bool wait_for(Connection &link, BaizeHostPhase phase)
{
    uint64_t deadline = monotonic_ms() + 1000;
    while (link.session.phase != phase) {
        if (phase == BAIZE_ACTIVE && link.session.phase != BAIZE_WAIT_ENABLED) return false;
        if (monotonic_ms() >= deadline || !exchange(link)) return false;
    }
    return true;
}

static bool jog(Connection &link)
{
    BaizeFrame command{};
    if (!baize_session_enable(&link.session, static_cast<uint32_t>(monotonic_ms()), &command) ||
        !serial_write_frame(link.port, &command) || !wait_for(link, BAIZE_ACTIVE)) return false;

    const int32_t target[BAIZE_WHEELS] = {500, 500, 500, 500}; // FL, FR, RL, RR in mrad/s.
    uint64_t deadline = monotonic_ms() + 300;
    while (monotonic_ms() < deadline) {
        if (!baize_session_target(&link.session, target) || !exchange(link)) return false;
    }
    return link.session.phase == BAIZE_ACTIVE;
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::fputs("Usage: chassis-example PORT (four wheels at +0.5 rad/s for 300 ms)\n", stderr);
        return 2;
    }
    for (int opening = 0; opening < 2; ++opening) {
        Connection link{};
        link.port = serial_open(argv[1]);
        if (link.port == SERIAL_INVALID) return 1;
        baize_session_open(&link.session, 250);
        bool ok = wait_for(link, BAIZE_READY);
        if (ok && opening == 0) {
            ok = jog(link);
            baize_session_stop(&link.session);
            if (!wait_for(link, BAIZE_READY)) ok = false;
        }
        serial_close(link.port);
        if (!ok) {
            std::fputs("Chassis I/O or motion handshake failed\n", stderr);
            return 1;
        }
    }
    std::puts("Stopped and reconnected with drive disabled.");
    return 0;
}
