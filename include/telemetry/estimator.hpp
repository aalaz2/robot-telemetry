#pragma once

#include <chrono>
#include <cmath>

#include "telemetry/types.hpp"

namespace telemetry {

// Simple event-driven dead reckoning. Every time a NEW measurement of
// either kind arrives: first PROPAGATE (x, y, heading) forward using
// whatever v/omega we currently believe, over the real elapsed time since
// the last update, THEN let the new measurement update our belief about
// v or omega for next time.
//
// Wheel readings inform forward speed; the gyro reading informs turn
// rate. That's the standard division of labor for cheap dead reckoning -
// wheel encoders are usually much better at "how far" than "which way",
// and gyros are the reverse.
//
// This class has NO idea what the true trajectory is - on a real robot
// there is no ground truth to peek at. That's deliberate: keeping this
// class ignorant of GroundTruthModel is what makes it something you could
// actually run on hardware, not just in simulation.
class DeadReckoningEstimator {
public:
    explicit DeadReckoningEstimator(double wheel_radius_m)
        : wheel_radius_(wheel_radius_m) {}

    void Update(const Measurement& m) {
        const Timestamp t = GetTimestamp(m);
        if (!initialized_) {
            last_update_ = t;
            initialized_ = true;
        }
        Propagate(t);
        std::visit([this](const auto& msg) { Apply(msg); }, m);
        last_update_ = t;
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double heading() const { return heading_; }

private:
    void Propagate(Timestamp t) {
        const double dt = std::chrono::duration<double>(t - last_update_).count();
        if (dt <= 0.0) {
            return;  // out-of-order or duplicate timestamp: ignore, don't corrupt state
        }
        x_ += v_ * std::cos(heading_) * dt;
        y_ += v_ * std::sin(heading_) * dt;
        heading_ += omega_ * dt;
    }

    void Apply(const WheelSpeedMeasurement& w) {
        v_ = wheel_radius_ * (w.left_rad_per_s + w.right_rad_per_s) / 2.0;
    }
    void Apply(const GyroMeasurement& g) {
        omega_ = g.angular_velocity_z_rad_s;
    }

    double wheel_radius_;
    bool initialized_ = false;
    Timestamp last_update_{};
    double x_ = 0.0, y_ = 0.0, heading_ = 0.0;
    double v_ = 0.0, omega_ = 0.0;
};

}  // namespace telemetry