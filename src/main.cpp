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
#include "telemetry/types.hpp"

using telemetry::BoundedQueue;
using telemetry::GetTimestamp;
using telemetry::GyroMeasurement;
using telemetry::Measurement;
using telemetry::Timestamp;
using telemetry::WheelSpeedMeasurement;

namespace {

struct SimConfig {
    double duration_s = 5.0;
    double wheel_rate_hz = 50.0;
    double gyro_rate_hz = 100.0;
    unsigned seed = 42;
    std::size_t queue_capacity = 64;
    unsigned consumer_delay_us = 0;  // artificial per-item processing delay
};

constexpr double kWheelBaseRadius = 3.0;  // rad/s nominal wheel speed
constexpr double kTurnAmplitude = 1.5;    // rad/s nominal yaw rate

void PrintUsage() {
    std::printf(
        "usage: telemetry_sim [--duration=SEC] [--wheel-rate=HZ] "
        "[--gyro-rate=HZ] [--seed=N] [--queue-capacity=N] "
        "[--consumer-delay-us=N]\n");
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

// Producer thread body: paces itself to real wall-clock time using
// sleep_until against a fixed start reference, so scheduling overhead on
// each iteration doesn't accumulate into drift over the run. (sleep_for(dt)
// called n times would drift; sleep_until(start + i*dt) does not.)
void WheelProducer(const SimConfig& cfg, unsigned seed,
                    BoundedQueue<Measurement>& queue) {
    std::mt19937 rng(seed);  // this thread's OWN rng - never shared
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

void GyroProducer(const SimConfig& cfg, unsigned seed,
                   BoundedQueue<Measurement>& queue) {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.01);
    const std::size_t n = SampleCount(cfg.duration_s, cfg.gyro_rate_hz);
    const auto start = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i) / cfg.gyro_rate_hz;
        std::this_thread::sleep_until(start + SecondsToTimestamp(t));

        const double yaw_rate =
            kTurnAmplitude * 0.1 * std::sin(0.5 * t) + noise(rng);
        queue.TryPush(Measurement{
            GyroMeasurement{SecondsToTimestamp(t), yaw_rate}});
    }
}

void PrintMeasurement(const Measurement& m) {
    std::visit(
        [](const auto& msg) {
            using T = std::decay_t<decltype(msg)>;
            if constexpr (std::is_same_v<T, WheelSpeedMeasurement>) {
                std::printf("WHEEL,%lld,%.4f,%.4f\n",
                            static_cast<long long>(msg.timestamp.count()),
                            msg.left_rad_per_s, msg.right_rad_per_s);
            } else {
                std::printf("GYRO,%lld,%.4f,\n",
                            static_cast<long long>(msg.timestamp.count()),
                            msg.angular_velocity_z_rad_s);
            }
        },
        m);
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

    std::printf("type,timestamp_ns,value1,value2\n");

    // Consumer: runs concurrently with both producers, blocking on the
    // queue between items.
    std::size_t processed = 0;
    std::thread consumer([&] {
        while (auto item = queue.Pop()) {
            if (cfg.consumer_delay_us > 0) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(cfg.consumer_delay_us));
            }
            PrintMeasurement(*item);
            ++processed;
        }
    });

    std::thread wheel_thread(WheelProducer, std::cref(cfg), cfg.seed,
                              std::ref(queue));
    std::thread gyro_thread(GyroProducer, std::cref(cfg), cfg.seed + 1,
                             std::ref(queue));

    // Clean shutdown: only signal "no more data" after BOTH producers have
    // genuinely finished generating. Calling Shutdown() any earlier would
    // tell the consumer to stop before all real data has arrived.
    wheel_thread.join();
    gyro_thread.join();
    queue.Shutdown();
    consumer.join();  // drains whatever's left, then returns

    const std::size_t wheel_count = SampleCount(cfg.duration_s, cfg.wheel_rate_hz);
    const std::size_t gyro_count = SampleCount(cfg.duration_s, cfg.gyro_rate_hz);
    const std::size_t total_generated = wheel_count + gyro_count;
    const std::size_t dropped = queue.DroppedCount();

    std::printf("\n--- summary ---\n");
    std::printf("generated: %zu wheel, %zu gyro (%zu total)\n", wheel_count,
                gyro_count, total_generated);
    std::printf("processed: %zu\n", processed);
    std::printf("dropped:   %zu\n", dropped);
    std::printf("invariant check (processed + dropped == generated): %s\n",
                (processed + dropped == total_generated) ? "OK" : "FAILED");

    return 0;
}