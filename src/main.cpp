#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>

#include "telemetry/bounded_queue.hpp"
#include "telemetry/csv_writer.hpp"
#include "telemetry/estimator.hpp"
#include "telemetry/ground_truth.hpp"
#include "telemetry/types.hpp"

using telemetry::BoundedQueue;
using telemetry::CsvWriter;
using telemetry::DeadReckoningEstimator;
using telemetry::GetTimestamp;
using telemetry::GyroMeasurement;
using telemetry::Measurement;
using telemetry::Timestamp;
using telemetry::WheelSpeedMeasurement;
using telemetry::sim::GroundTruthModel;

namespace {

struct SimConfig {
    double duration_s = 5.0;
    double wheel_rate_hz = 50.0;
    double gyro_rate_hz = 100.0;
    unsigned seed = 42;
    std::size_t queue_capacity = 64;
    unsigned consumer_delay_us = 0;
    std::string output_path = "recording.csv";
};

constexpr double kWheelBaseRadius = 3.0;  // rad/s nominal wheel speed
// Physical robot dimensions (roughly a small 2WD hobby chassis) - used to
// turn angular wheel speeds into real-world linear velocity and turn rate.
constexpr double kWheelRadiusM = 0.033;
constexpr double kTrackWidthM = 0.15;

void PrintUsage() {
    std::printf(
        "usage: telemetry_sim [--duration=SEC] [--wheel-rate=HZ] "
        "[--gyro-rate=HZ] [--seed=N] [--queue-capacity=N] "
        "[--consumer-delay-us=N] [--output=PATH]\n");
}

enum class ArgStatus { kOk, kHelp, kError };

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
            return {ArgStatus::kHelp, cfg};
        }
        const auto eq = arg.find('=');
        if (eq == std::string_view::npos) {
            std::fprintf(stderr, "bad argument (expected --flag=value): %s\n",
                         argv[i]);
            return {ArgStatus::kError, cfg};
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
            } else if (key == "--queue-capacity") {
                cfg.queue_capacity = static_cast<std::size_t>(std::stoul(value));
            } else if (key == "--consumer-delay-us") {
                cfg.consumer_delay_us = static_cast<unsigned>(std::stoul(value));
            } else if (key == "--output") {
                cfg.output_path = value;
            } else {
                std::fprintf(stderr, "unknown flag: %s\n",
                             std::string(key).c_str());
                return {ArgStatus::kError, cfg};
            }
        } catch (const std::exception&) {
            std::fprintf(stderr, "invalid value for %s: %s\n",
                         std::string(key).c_str(), value.c_str());
            return {ArgStatus::kError, cfg};
        }
    }
    if (cfg.duration_s <= 0.0 || cfg.wheel_rate_hz <= 0.0 ||
        cfg.gyro_rate_hz <= 0.0 || cfg.queue_capacity == 0) {
        std::fprintf(stderr, "duration, rates, and queue-capacity must be positive\n");
        return {ArgStatus::kError, cfg};
    }
    return {ArgStatus::kOk, cfg};
}

Timestamp SecondsToTimestamp(double seconds) {
    return std::chrono::duration_cast<Timestamp>(
        std::chrono::duration<double>(seconds));
}

std::size_t SampleCount(double duration_s, double rate_hz) {
    return static_cast<std::size_t>(std::floor(duration_s * rate_hz)) + 1;
}

void WheelProducer(const SimConfig& cfg, unsigned seed,
                    BoundedQueue<Measurement>& queue) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.02);
    const std::size_t n = SampleCount(cfg.duration_s, cfg.wheel_rate_hz);
    const auto start = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / cfg.wheel_rate_hz;
        std::this_thread::sleep_until(start + SecondsToTimestamp(t));

        const double left = kWheelBaseRadius + noise(rng);
        const double right = kWheelBaseRadius * 0.9 + noise(rng);
        queue.TryPush(Measurement{
            WheelSpeedMeasurement{SecondsToTimestamp(t), left, right}});
    }
}

// UPDATED for Day 5-6: the true yaw rate the gyro measures is now derived
// from the SAME nominal wheel speeds as WheelProducer, via the same
// kinematics the estimator uses - not an independent sine wave. The two
// sensors now genuinely disagree only because of their separate noise
// draws, which is what makes a trajectory-error metric meaningful: there
// is exactly one true motion, and both sensors are noisy measurements
// of it.
void GyroProducer(const SimConfig& cfg, unsigned seed,
                   BoundedQueue<Measurement>& queue) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.01);
    const std::size_t n = SampleCount(cfg.duration_s, cfg.gyro_rate_hz);
    const auto start = std::chrono::steady_clock::now();

    const double left_true = kWheelBaseRadius;
    const double right_true = kWheelBaseRadius * 0.9;
    const double true_omega =
        kWheelRadiusM * (right_true - left_true) / kTrackWidthM;

    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / cfg.gyro_rate_hz;
        std::this_thread::sleep_until(start + SecondsToTimestamp(t));

        const double yaw_rate = true_omega + noise(rng);
        queue.TryPush(Measurement{
            GyroMeasurement{SecondsToTimestamp(t), yaw_rate}});
    }
}

}  // namespace

int main(int argc, char** argv) {
    const ParseResult parsed = ParseArgs(argc, argv);
    if (parsed.status == ArgStatus::kHelp) {
        return EXIT_SUCCESS;
    }
    if (parsed.status == ArgStatus::kError) {
        return EXIT_FAILURE;
    }
    const SimConfig cfg = parsed.config;

    BoundedQueue<Measurement> queue(cfg.queue_capacity);
    DeadReckoningEstimator estimator(kWheelRadiusM);
    CsvWriter csv(cfg.output_path);
    csv.WriteHeader();

    // Consumer: pulls from the queue, updates the estimator, records both
    // the raw reading and the estimator's resulting pose to CSV.
    std::size_t processed = 0;
    std::thread consumer([&] {
        while (auto item = queue.Pop()) {
            if (cfg.consumer_delay_us > 0) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(cfg.consumer_delay_us));
            }
            estimator.Update(*item);
            std::visit(
                [&](const auto& msg) {
                    using T = std::decay_t<decltype(msg)>;
                    const long long ts =
                        static_cast<long long>(msg.timestamp.count());
                    if constexpr (std::is_same_v<T, WheelSpeedMeasurement>) {
                        csv.WriteRow("WHEEL", ts, msg.left_rad_per_s,
                                     msg.right_rad_per_s, estimator.x(),
                                     estimator.y(), estimator.heading());
                    } else {
                        csv.WriteRow("GYRO", ts, msg.angular_velocity_z_rad_s,
                                     0.0, estimator.x(), estimator.y(),
                                     estimator.heading());
                    }
                },
                *item);
            ++processed;
        }
    });

    std::thread wheel_thread(WheelProducer, std::cref(cfg), cfg.seed,
                              std::ref(queue));
    std::thread gyro_thread(GyroProducer, std::cref(cfg), cfg.seed + 1,
                             std::ref(queue));

    wheel_thread.join();
    gyro_thread.join();
    queue.Shutdown();
    consumer.join();

    const std::size_t wheel_count = SampleCount(cfg.duration_s, cfg.wheel_rate_hz);
    const std::size_t gyro_count = SampleCount(cfg.duration_s, cfg.gyro_rate_hz);
    const std::size_t total_generated = wheel_count + gyro_count;
    const std::size_t dropped = queue.DroppedCount();

    const GroundTruthModel truth{kWheelRadiusM, kTrackWidthM, kWheelBaseRadius,
                                  kWheelBaseRadius * 0.9};
    double true_x = 0.0, true_y = 0.0, true_heading = 0.0;
    truth.PoseAt(cfg.duration_s, true_x, true_y, true_heading);
    const double final_error =
        std::hypot(estimator.x() - true_x, estimator.y() - true_y);

    std::printf("\n--- summary ---\n");
    std::printf("generated: %zu wheel, %zu gyro (%zu total)\n", wheel_count,
                gyro_count, total_generated);
    std::printf("processed: %zu\n", processed);
    std::printf("dropped:   %zu\n", dropped);
    std::printf("invariant check (processed + dropped == generated): %s\n",
                (processed + dropped == total_generated) ? "OK" : "FAILED");
    std::printf("estimated final pose: x=%.4f y=%.4f heading=%.4f rad\n",
                estimator.x(), estimator.y(), estimator.heading());
    std::printf("true final pose:      x=%.4f y=%.4f heading=%.4f rad\n",
                true_x, true_y, true_heading);
    std::printf("final position error: %.4f m (full trajectory RMSE: Day 9-10)\n",
                final_error);
    std::printf("wrote %s\n", cfg.output_path.c_str());

    return 0;
}