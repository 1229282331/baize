#ifndef BAIZE_CHASSIS_H
#define BAIZE_CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct baize_chassis baize_chassis;

typedef enum {
    BAIZE_FRONT_LEFT, BAIZE_FRONT_RIGHT, BAIZE_REAR_LEFT, BAIZE_REAR_RIGHT, BAIZE_WHEEL_COUNT
} baize_wheel;

typedef enum {
    BAIZE_DRIVE_DISABLED, BAIZE_DRIVE_ENABLED, BAIZE_DRIVE_FAULTED
} baize_drive_state;

typedef enum {
    BAIZE_STOP_NONE, BAIZE_STOP_RESET, BAIZE_STOP_REQUEST, BAIZE_STOP_TIMEOUT, BAIZE_STOP_FAULT
} baize_stop_reason;

typedef struct {
    double wheel_radius_m;
    double track_width_m;
} baize_geometry;

typedef struct {
    double rad_s[BAIZE_WHEEL_COUNT];  /* Positive rolls the vehicle forward. */
} baize_wheel_speeds;

typedef struct {
    uint32_t sample_ms;              /* Board time, wraps modulo 2^32. */
    bool valid;                      /* A new wheel sample arrived within 250 ms. */
    uint8_t state;                   /* baize_drive_state */
    uint8_t stop_reason;
    double supply_v;                 /* NAN until the ADC has a valid reading. */
    double angle_rad[BAIZE_WHEEL_COUNT];
    double velocity_rad_s[BAIZE_WHEEL_COUNT];
} baize_drive_sample;

typedef struct {
    uint32_t sample_ms;
    bool valid;                      /* Independent of wheel feedback. */
    double acceleration_m_s2[3];
    double angular_velocity_rad_s[3];
} baize_imu_sample;

typedef struct {
    baize_drive_sample drive;
    baize_imu_sample imu;
} baize_chassis_state;

/* Geometry conversion does no I/O. Invalid geometry or a nonfinite input yields NAN targets. */
baize_geometry baize_nominal_geometry(void);
baize_wheel_speeds baize_to_wheel_speeds(baize_geometry geometry, double forward_m_s, double yaw_rate_rad_s);

/* One caller owns a handle and serializes its calls. Open waits up to 1 s for stopped
 * feedback. A latched board fault also opens for diagnostics. NULL indicates failure. */
baize_chassis *baize_chassis_open(const char *port);
void baize_chassis_close(baize_chassis *chassis);

/* Enable and disable wait for board confirmation. Close sends a best-effort STOP. */
bool baize_chassis_enable(baize_chassis *chassis);
bool baize_chassis_disable(baize_chassis *chassis);

/* Consume already available bytes and return the latest snapshot. Check each valid flag.
 * An I/O failure returns false, invalidates both samples and requires reopening the port. */
bool baize_chassis_read(baize_chassis *chassis, baize_chassis_state *state);

/* Submit once while enabled with fresh wheel feedback. Each wheel must be finite and
 * within [-2, 2] rad/s. Continue motion by reading and writing each control period. */
bool baize_chassis_write(baize_chassis *chassis, const baize_wheel_speeds *wheels);

#ifdef __cplusplus
}
#endif
#endif
