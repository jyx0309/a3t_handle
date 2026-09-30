#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

// Non-blocking startup guard. No gain increase or extra feed-forward torque.
class MitStartupHold {
 public:
  void reset(const std::vector<double>& target = {}) {
    target_ = target;
    release_since_ = -1.0;
    last_time_ = -1.0;
  }
  bool active() const { return !target_.empty(); }
  // Velocity safety is checked by SafetyMonitor against configured robot limits.
  // Otherwise return the fixed-target weight (1 = hold, 0 = normal handle).
  double update(double t, const std::vector<double>& q, const std::vector<double>& dq) {
    if (!active()) return 0.0;
    if (!std::isfinite(t) || t < 0 || t < last_time_ || q.size() != target_.size() ||
        dq.size() != target_.size()) return -1.0;
    last_time_ = t;
    for (size_t i = 0; i < q.size(); ++i) {
      if (!std::isfinite(q[i]) || !std::isfinite(dq[i]) || !std::isfinite(target_[i]) ||
          std::abs(q[i] - target_[i]) > 0.03) return -1.0;
    }
    if (release_since_ < 0) {
      // Fixed short hold; milliradian residuals are not a disable condition.
      if (t >= 0.5) release_since_ = 0.5;
    }
    if (release_since_ < 0) return 1.0;
    const double u = std::clamp((t - release_since_) / 0.5, 0.0, 1.0);
    if (u >= 1.0) { reset(); return 0.0; }
    return 1.0 - u * u * (3.0 - 2.0 * u);
  }
  void apply(double weight, std::vector<double>& position) const {
    if (weight <= 0 || position.size() != target_.size()) return;
    for (size_t i = 0; i < position.size(); ++i)
      position[i] += weight * (target_[i] - position[i]);
  }
 private:
  std::vector<double> target_;
  double release_since_{-1.0}, last_time_{-1.0};
};
