#pragma once

#include <cstdio>
#include <stdexcept>
#include <string>

namespace telemetry {

// A minimal CSV writer. Single-threaded use only - the queue design
// already guarantees exactly one consumer thread ever calls this, so
// there's no locking here. If that assumption ever changes, this class
// needs a mutex before it's used from more than one thread.
class CsvWriter {
public:
    explicit CsvWriter(const std::string& path)
        : file_(std::fopen(path.c_str(), "w")) {
        if (!file_) {
            throw std::runtime_error("failed to open CSV file for writing: " +
                                      path);
        }
    }

    ~CsvWriter() {
        if (file_) {
            std::fclose(file_);
        }
    }

    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    void WriteHeader() {
        std::fprintf(file_,
                      "type,timestamp_ns,value1,value2,est_x,est_y,"
                      "est_heading\n");
    }

    // 9 decimal digits: far more precision than the sensor noise model
    // has any right to claim, but enough that re-parsing these numbers
    // back during replay doesn't itself introduce meaningfully different
    // rounding than the live run had. See the write-up on why this
    // precision choice matters for "deterministic" replay.
    void WriteRow(const char* type, long long timestamp_ns, double value1,
                  double value2, double est_x, double est_y,
                  double est_heading) {
        std::fprintf(file_, "%s,%lld,%.9f,%.9f,%.9f,%.9f,%.9f\n", type,
                     timestamp_ns, value1, value2, est_x, est_y,
                     est_heading);
    }

private:
    std::FILE* file_;
};

}  // namespace telemetry