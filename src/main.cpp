#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <random>
#include <string>
#include <string_view>
#include <vector>

#include "telemetry/types.hpp"

using telemetry::GyroMeasurment;
using telemetry::Timestamp;
using telemetry::WheelSpeedMeasurement;

namespace {
    
    struct SimConfig {
        double duration_s = 5.0;
        double wheel_rate_hz = 50.0;
        double gyro_rate_hz = 100.0;
        unsigned seed = 42;
    };

    constexpr double kWheelBaseRadius = 3.0; //rad/s nominla wheel speed
    constexpr double kTurnAmplitude = 1.5; // rad/s nominal yaw rate

    void PrintUsage() {
        std::printf(
            "usage: telemetry_sim [--duration=SEC] [--wheel-rate=HZ] "
            "[--gyro-rate=HZ] [--seed=N]\n"
        );
    }

    enum class ArgStatus {kOk, kHelp, kError };

    struct ParseResult {
        ArgStatus status = ArgStatus::kOk;
        SimConfig config;
    };

    ParseResult ParseArgs(int argc, char** argv) {
        SimConfig cfg;
        for (int i = 1; i < argc; ++i) {
            const std::string_view arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                PrintUsage();
                return {
                    ArgStatus::kHelp, cfg
                };
            }
            const auto eq = arg.find('=');
            if (eq == std::string_view::npos) {
                std::fprintf(stderr, "bad argument (expected --flag=value): %s\n", argv[i]);
                return {
                    ArgStatus::kError, cfg
                };
            }
            const std::string_view key = arg.substr(0, eq);
            const std::string value(arg.substr(eq + 1));
            try {
                if (key == "--duration") {
                    cfg.duration_s = std::stod(value);
                } else if (key == "--wheel-rate") {
                    cfg.wheel_rate_hz = std::stod(value);
                } else if (key == "--gyro-rate") {
                    cfg.gyro_rate_hz = std::stod(value);
                } else if (key == "--seed") {
                    cfg.seed = static_cast<unsigned>(std::stoul(value));
                } else {
                    std::fprintf(stderr, "unknown flag: %s\n",
                             std::string(key).c_str());
                    return {ArgStatus::kError, cfg};
                }
            } catch (const std::exception&) {
                std::fprintf(stderr, "invalid value for %s: %s\n",
                            std::string(key).c_str(), value.c_str());
                return {
                    ArgStatus::kError, cfg
                };
            }
        }
        if (cfg.duration_s <= 0.0 || cfg.wheel_rate_hz <= 0.0 || 
            cfg.gyro_rate_hz <= 0.0) {
            std::fprintf(stderr, "duration and rates must be positive\n");
            return {
                ArgStatus::kError, cfg
            };
        }
        return {ArgStatus::kOk, cfg};
    }
    
    Timestamp SecondsToTimestamp(double seconds) {
        return std::chrono::duration_cast<Timestamp>(
            std::chrono::duration<double>(seconds));
    }

    // Number of samples for a fixed-rate stream over [0, duration_s).
    // Computed once, up front, instead of accumulating a floating-point
    // counter across the loop - see the write-up on why that matters.
    std::size_t SampleCount(double duration_s, double rate_hz) {
        return static_cast<std::size_t>(std::floor(duration_s * rate_hz)) + 1;
    }
    //fix here
    std::vector<WheelSpeedMeasurement> GenerateWheelSpeeds(
        const SimConfig& cfg, std::mt19937& rng) {
        std::normal_distribution<double> noise(0.0, 0.02);
        std::vector<WheelSpeedMeasurement> out;
        const std::size_t n = SampleCount(cfg.duration_s, cfg.wheel_rate_hz);
        out.reserve(n);
        for(std::size_t i = 0; i < n; ++i) {
            // slight left/right asymmetry so the robot gently turns -
            //  gives the estimator something nontrivial to integrate later.
            const double t = static_cast<double>(i) / cfg.wheel_rate_hz;
            const double left = kWheelBaseRadius + noise(rng);
            const double right = kWheelBaseRadius * 0.9 + noise(rng);
            out.push_back({SecondsToTimestamp(t), left, right});
        }
        return out;
    }

    std::vector<GyroMeasurment> GenerateGyro(
        const SimConfig& cfg, std::mt19937& rng) {
        std::normal_distribution<double> noise(0.0, 0.01);
        std::vector<GyroMeasurment> out;
        const std::size_t n = SampleCount(cfg.duration_s, cfg.gyro_rate_hz);
        out.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const double t = static_cast<double>(i) / cfg.gyro_rate_hz;
            const double yaw_rate =
                kTurnAmplitude * 0.1 * std::sin(0.5 * t) + noise(rng);
            out.push_back({SecondsToTimestamp(t), yaw_rate});
        }
        return out;
    }

} //namespace end

int main(int argc, char** argv) {
    const ParseResult parsed = ParseArgs(argc, argv);
    if (parsed.status == ArgStatus::kHelp) {
        return EXIT_SUCCESS;
    }
    if (parsed.status == ArgStatus::kError) {
        return EXIT_FAILURE;
    }
    const SimConfig cfg = parsed.config;

    std::mt19937 rng(42); // fixed seed: reproducible

    const auto wheel = GenerateWheelSpeeds(cfg, rng);
    const auto gyro = GenerateGyro(cfg, rng);

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
