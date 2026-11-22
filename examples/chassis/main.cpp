#include <baize/chassis.hpp>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

static bool drive(baize::Chassis &chassis, const baize::WheelSpeeds &wheels)
{
    if (!chassis.enable()) return false;
    baize::ChassisState state{};
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    bool ok = true;
    while (std::chrono::steady_clock::now() < deadline) {
        if (!chassis.read(state) || !state.drive.valid || !chassis.write(wheels)) {
            ok = false;
            break;
        }
        std::printf("wheel %u ms, %.3f V, rad/s", static_cast<unsigned>(state.drive.sample_ms),
                    state.drive.supply_v);
        for (double speed : state.drive.velocity_rad_s) std::printf(" %.3f", speed);
        if (state.imu.valid)
            std::printf("  imu %u ms, az %.3f m/s^2, gz %.3f rad/s",
                        static_cast<unsigned>(state.imu.sample_ms), state.imu.acceleration_m_s2[2],
                        state.imu.angular_velocity_rad_s[2]);
        std::puts("");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return chassis.disable() && ok;
}

int main(int argc, char **argv)
{
    bool turn = argc == 3 && std::strcmp(argv[2], "--turn") == 0;
    if (argc != 2 && !turn) {
        std::fputs("Usage: chassis-example PORT [--turn] (300 ms wheel motion)\n", stderr);
        return 2;
    }
    baize::Chassis chassis;
    const auto geometry = baize::nominal_geometry();
    const auto wheels = baize::to_wheel_speeds(geometry, turn ? 0 : geometry.wheel_radius_m * 0.5,
                                                turn ? geometry.wheel_radius_m / geometry.track_width_m : 0);
    if (!chassis.open(argv[1]) || !drive(chassis, wheels)) return 1;
    chassis.close();

    baize::ChassisState state{};
    if (!chassis.open(argv[1]) || !chassis.read(state) || !state.drive.valid ||
        state.drive.state != baize::DriveState::Disabled) return 1;
    std::puts("Stopped and reconnected with drive disabled.");
    return 0;
}
