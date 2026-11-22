#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif
#include <baize/chassis.h>

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

static void wait_one_period(void)
{
#ifdef _WIN32
    Sleep(20);
#else
    nanosleep(&(struct timespec){.tv_nsec = 20000000}, NULL);
#endif
}

int main(int argc, char **argv)
{
    bool turn = argc == 3 && strcmp(argv[2], "--turn") == 0;
    if (argc != 2 && !turn) {
        fputs("Usage: chassis-c-example PORT [--turn] (15 wheel commands at 20 ms)\n", stderr);
        return 2;
    }
    baize_geometry geometry = baize_nominal_geometry();
    baize_wheel_speeds wheels = baize_to_wheel_speeds(geometry, turn ? 0 : geometry.wheel_radius_m * 0.5,
                                                       turn ? geometry.wheel_radius_m / geometry.track_width_m : 0);
    baize_chassis *chassis = baize_chassis_open(argv[1]);
    if (!chassis) return 1;
    baize_chassis_state state;
    bool ok = baize_chassis_enable(chassis);
    for (unsigned i = 0; ok && i < 15; ++i) {
        ok = baize_chassis_read(chassis, &state) && state.drive.valid && baize_chassis_write(chassis, &wheels);
        if (!ok) break;
        printf("wheel %u ms, %.3f V, rad/s", (unsigned)state.drive.sample_ms, state.drive.supply_v);
        for (unsigned j = 0; j < BAIZE_WHEEL_COUNT; ++j) printf(" %.3f", state.drive.velocity_rad_s[j]);
        if (state.imu.valid) printf("  imu %u ms, az %.3f m/s^2, gz %.3f rad/s",
                                   (unsigned)state.imu.sample_ms, state.imu.acceleration_m_s2[2],
                                   state.imu.angular_velocity_rad_s[2]);
        puts("");
        wait_one_period();
    }
    ok = baize_chassis_disable(chassis) && ok;
    baize_chassis_close(chassis);
    if (!ok) return 1;
    chassis = baize_chassis_open(argv[1]);
    ok = baize_chassis_read(chassis, &state) && state.drive.valid && state.drive.state == BAIZE_DRIVE_DISABLED;
    baize_chassis_close(chassis);
    if (ok) puts("Stopped and reconnected with drive disabled.");
    return ok ? 0 : 1;
}
