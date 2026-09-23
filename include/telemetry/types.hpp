#pragma once
#include <variant>
#include <chrono>
#include <cstdint>\

namespace telemetry {
    //timestamp and simclock lets recorded run at any speed
    using SimClock = std::chrono::steady_clock;
    using Timestamp = std::chrono::nanoseconds;

    struct WheelSpeedMeasurement {
        Timestamp timestamp{};
        double left_rad_per_s{};
        double right_rad_per_s{};
    };

    struct GyroMeasurement {
        Timestamp timestamp{};
        double angular_velocity_z_rad_s{}; //yaw rate?
    };

    using Measurement = std::variant<WheelSpeedMeasurement, GyroMeasurement>;

    inline Timestamp GetTimestamp(const Measurement& m) {
        return std::visit([](const auto& msg) { return msg.timestamp; }, m);
    }
} //namespace end

