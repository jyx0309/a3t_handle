#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <utility>

// Same cubic path and 0.1 s duration rounding as the original PV entry.
// Only sampling density changes; the nominal speed and duration do not.
struct PvTrajectory {
  std::vector<double> start, goal;
  double duration_s{0.1};
  PvTrajectory(std::vector<double> from, std::vector<double> to) : start(std::move(from)), goal(std::move(to)) {
    for (size_t i = 0; i < start.size(); ++i)
      duration_s = std::max(duration_s, std::ceil(1.5 * std::abs(goal[i]-start[i]) / (0.1 * 0.1)) * 0.1);
  }
  void sample(double elapsed_s, std::vector<double>& q, std::vector<double>& dq) const {
    const double u = std::clamp(elapsed_s / duration_s, 0.0, 1.0);
    const double blend = u*u*(3.0-2.0*u);
    const double rate = 6.0*u*(1.0-u)/duration_s;
    q.resize(start.size()); dq.resize(start.size());
    for (size_t i=0; i<start.size(); ++i) {
      q[i] = start[i] + blend*(goal[i]-start[i]);
      dq[i] = rate*(goal[i]-start[i]);
    }
  }
  // Skip missed deadlines instead of bursting stale trajectory samples.
  static long long nextDeadlineNs(long long previous, long long now) {
    constexpr long long period = 10'000'000;
    return std::max(previous + period, (now / period + 1) * period);
  }
};
