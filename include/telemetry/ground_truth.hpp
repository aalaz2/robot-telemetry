#pragma once

#include <cmath>

namespace telemetry::sim {

// Models the TRUE, noiseless motion of the SIMULATED robot. A real onboard
// estimator never has access to this - it exists only so the test harness
// (and you) can measure how far off the estimator's guess actually is.
//
// Deliberately kept OUT of include/telemetry's main namespace and out of
// DeadReckoningEstimator entirely: this struct represents information a
// real robot's software would never have. Mixing it into the estimator
// would make it impossible to tell, just by reading the estimator's code,
// whether it would actually work on real hardware.
struct GroundTruthModel {
    double wheel_radius_m;
    double track_width_m;
    double left_true_rad_s;
    double right_true_rad_s;

    double LinearVelocity() const {
        return wheel_radius_m * (left_true_rad_s + right_true_rad_s) / 2.0;
    }

    double YawRate() const {
        return wheel_radius_m * (right_true_rad_s - left_true_rad_s) /
               track_width_m;
    }

    // Closed-form pose at time t (seconds since start, robot starts at the
    // origin facing heading 0). Exact - no numerical integration needed,
    // because v and omega are constant for this trajectory.
    void PoseAt(double t, double& x, double& y, double& heading) const {
        const double v = LinearVelocity();
        const double w = YawRate();
        heading = w * t;
        if (std::abs(w) < 1e-9) {
            x = v * t;
            y = 0.0;
        } else {
            x = (v / w) * std::sin(heading);
            y = (v / w) * (1.0 - std::cos(heading));
        }
    }
};

}  // namespace telemetry::sim