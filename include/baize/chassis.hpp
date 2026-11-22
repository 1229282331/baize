#ifndef BAIZE_CHASSIS_HPP
#define BAIZE_CHASSIS_HPP

#include "chassis.h"

namespace baize {

using Geometry = baize_geometry;
using WheelSpeeds = baize_wheel_speeds;
using DriveSample = baize_drive_sample;
using ImuSample = baize_imu_sample;
using ChassisState = baize_chassis_state;

enum DriveState : uint8_t {
    Disabled = BAIZE_DRIVE_DISABLED, Enabled = BAIZE_DRIVE_ENABLED, Faulted = BAIZE_DRIVE_FAULTED
};
enum StopReason : uint8_t {
    Running = BAIZE_STOP_NONE, BoardReset = BAIZE_STOP_RESET, Requested = BAIZE_STOP_REQUEST,
    CommandTimeout = BAIZE_STOP_TIMEOUT, DriveFault = BAIZE_STOP_FAULT
};

inline Geometry nominal_geometry() { return baize_nominal_geometry(); }

inline WheelSpeeds to_wheel_speeds(Geometry geometry, double forward_m_s, double yaw_rate_rad_s)
{
    return baize_to_wheel_speeds(geometry, forward_m_s, yaw_rate_rad_s);
}

class Chassis {
public:
    Chassis() = default;
    ~Chassis() { close(); }
    Chassis(const Chassis &) = delete;
    Chassis &operator=(const Chassis &) = delete;

    bool open(const char *port)
    {
        if (handle_) return false;
        handle_ = baize_chassis_open(port);
        return handle_ != nullptr;
    }

    void close()
    {
        baize_chassis_close(handle_);
        handle_ = nullptr;
    }

    bool enable() { return baize_chassis_enable(handle_); }
    bool disable() { return baize_chassis_disable(handle_); }
    bool read(ChassisState &state) { return baize_chassis_read(handle_, &state); }
    bool write(const WheelSpeeds &wheels) { return baize_chassis_write(handle_, &wheels); }

private:
    baize_chassis *handle_ = nullptr;
};

}  // namespace baize
#endif
