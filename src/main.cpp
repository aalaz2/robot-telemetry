#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "telemetry/types.hpp"

using telemetry::GyroMeasurment;
using telemetry::Timestamp;
using telemetry::WheelSpeedMeasurement;

namespace {
    
    constexpr double kSimDurationS = 5.0;
    constexpr double kWheelRateHz = 50.0;
    constexpr double kGyroRateHz = 100.0;
    constexpr double kWheelBaseRadius = 3.0; //rad/s nominla wheel speed
    constexpr double kTurnAmplitude = 1.5; // rad/s nominal yaw rate

    Timestamp SecondsToTimestamp(double seconds) {
        return std::chrono::duration_cast<Timestamp>(
            std::chrono::duration<double>(seconds));
    }

    std::vector<WheelSpeedMeasurement> GenerateWheelSpeeds(std::mt19937& rng) {
        std::normal_distribution<double> noise(0.0, 0.02);
        std::vector<WheelSpeedMeasurement> out;
        const double dt = 1.0 / kWheelRateHz;
        for(double t = 0.0; t < kSimDurationS; t += dt){
            // slight left/right asymmetry so the robot gently turns -
            //  gives the estimator something nontrivial to integrate later.
            const double left = kWheelBaseRadius + noise(rng);
            const double right = kWheelBaseRadius * 0.9 + noise(rng);
            out.push_back({SecondsToTimestamp(t), left, right});
        }
        return out;
    }

    std::vector<GyroMeasurment> GenerateGyro(std::mt19937& rng) {
        std::normal_distribution<double> noise(0.0, 0.01);
        std::vector<GyroMeasurment> out;
        const double dt = 1.0 / kGyroRateHz;
        for(double t = 0.0; t < kSimDurationS; t += dt) {
            const double yaw_rate = 
                kTurnAmplitude * 0.1 * std::sin(0.5 * t) + noise(rng);
            out.push_back({SecondsToTimestamp(t), yaw_rate});
        }
        return out;
    }

} //namespace end

int main() {
    std::mt19937 rng(42); // fixed seed: reproducible

    const auto wheel = GenerateWheelSpeeds(rng);
    const auto gyro = GenerateGyro(rng);

    std::printf("generate %zu wheel-speed samples, %zu gyro samples\n",
                 wheel.size(), gyro.size());
    
    std::size_t wi = 0, gi = 0;

    std::printf("type, timestamp_ns, value1, value2\n");
    while(wi < wheel.size() || gi < gyro.size()) {
        const bool take_wheel = 
            gi >= gyro.size() ||
            (wi < wheel.size() && 
            wheel[wi].timestamp < gyro[gi].timestamp);
        
        if (take_wheel) {
            const auto& m = wheel[wi++];
            std::printf("WHEEL, %lld, %.4f, %.4f\n",
                        static_cast<long long>(m.timestamp.count()),
                        m.left_rad_per_s, m.right_rad_per_s);
        } else {
            const auto& m = gyro[gi++];
            std::printf("GYRO,%lld,%.4f,\n",
                        static_cast<long long>(m.timestamp.count()),
                        m.angular_velocity_z_rad_s);
        }
    }

    return 0;
}
