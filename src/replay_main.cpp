#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "telemetry/estimator.hpp"
#include "telemetry/types.hpp"

using telemetry::DeadReckoningEstimator;
using telemetry::GyroMeasurement;
using telemetry::Measurement;
using telemetry::Timestamp;
using telemetry::WheelSpeedMeasurement;

namespace {

constexpr double kWheelRadiusM = 0.033;  // must match main.cpp's constant

struct Row {
    std::string type;
    long long timestamp_ns = 0;
    double value1 = 0.0, value2 = 0.0;
    double recorded_x = 0.0, recorded_y = 0.0, recorded_heading = 0.0;
};

bool ParseRow(const std::string& line, Row& row) {
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        fields.push_back(field);
    }
    if (fields.size() != 7) {
        return false;
    }
    try {
        row.type = fields[0];
        row.timestamp_ns = std::stoll(fields[1]);
        row.value1 = std::stod(fields[2]);
        row.value2 = std::stod(fields[3]);
        row.recorded_x = std::stod(fields[4]);
        row.recorded_y = std::stod(fields[5]);
        row.recorded_heading = std::stod(fields[6]);
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: telemetry_replay <recording.csv>\n");
        return EXIT_FAILURE;
    }

    std::ifstream in(argv[1]);
    if (!in) {
        std::fprintf(stderr, "failed to open %s\n", argv[1]);
        return EXIT_FAILURE;
    }

    std::string line;
    std::getline(in, line);  // skip header row

    // A fresh estimator, single-threaded, no queue, no producers - fed the
    // exact same measurements, in the exact same order, that the live run
    // recorded. If dead reckoning is genuinely deterministic, this
    // reproduces the live run's trajectory to within floating-point
    // round-trip precision.
    DeadReckoningEstimator estimator(kWheelRadiusM);

    std::size_t row_count = 0;
    std::size_t malformed = 0;
    std::size_t mismatches = 0;
    double max_deviation = 0.0;
    constexpr double kTolerance = 1e-6;  // meters; see write-up on why not 0

    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        Row row;
        if (!ParseRow(line, row)) {
            std::fprintf(stderr, "skipping malformed row %zu: %s\n",
                         row_count, line.c_str());
            ++malformed;
            continue;
        }

        Measurement m;
        if (row.type == "WHEEL") {
            m = WheelSpeedMeasurement{Timestamp(row.timestamp_ns), row.value1,
                                       row.value2};
        } else if (row.type == "GYRO") {
            m = GyroMeasurement{Timestamp(row.timestamp_ns), row.value1};
        } else {
            std::fprintf(stderr, "skipping unknown type in row %zu: %s\n",
                         row_count, row.type.c_str());
            ++malformed;
            continue;
        }

        estimator.Update(m);

        const double deviation = std::hypot(estimator.x() - row.recorded_x,
                                             estimator.y() - row.recorded_y);
        max_deviation = std::max(max_deviation, deviation);
        if (deviation > kTolerance) {
            ++mismatches;
        }
        ++row_count;
    }

    std::printf("replayed %zu measurements (%zu malformed rows skipped)\n",
                row_count, malformed);
    std::printf("max positional deviation from recorded run: %.9f m\n",
                max_deviation);
    std::printf("rows exceeding %.1e m tolerance: %zu\n", kTolerance,
                mismatches);
    std::printf("deterministic replay: %s\n",
                mismatches == 0 ? "MATCH" : "MISMATCH");

    return mismatches == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}