#pragma once
#include <cmath>
#include <algorithm>
#include <vector>
#include "friction_probe.hpp"

// J1 (+,-) x3, then J2 ... J6. Never retry a failed direction.
class FrictionBatch {
 public:
  enum class Phase { Idle, Probing, Releasing, Settling };
  Phase phase{Phase::Idle};
  std::vector<double> thresholds;
  std::vector<int> identified;
  double release_torque{0};
  bool active() const { return phase!=Phase::Idle; }
  int index() const { return static_cast<int>(thresholds.size()); }
  int joint() const { return index()/6; }
  int direction() const { return index()%2==0?1:-1; }
  int repetition() const { return (index()%6)/2+1; }
  const std::vector<double>& origin() const { return origin_; }
  bool withinOrigin(const std::vector<double>& q) const {
    if (origin_.size()!=6 || q.size()!=6) return false;
    for (size_t i=0;i<6;++i)
      if (!std::isfinite(q[i]) || std::abs(q[i]-origin_[i])>0.05) return false;
    return true;
  }
  bool start(const std::vector<double>& q = {}) {
    if (active()) return false;
    if (!q.empty()) {
      if (q.size()!=6) return false;
      for (double x:q) if (!std::isfinite(x)) return false;
    }
    origin_=q;
    thresholds.clear(); identified.clear(); release_torque=0; phase=Phase::Probing; return true;
  }
  bool accept(double threshold, double remaining_torque) {
    if (phase!=Phase::Probing || index()>=36 || !std::isfinite(threshold) ||
        threshold*direction()<=0 || std::abs(threshold)>FrictionProbe::limit_nm+1e-6 ||
        !std::isfinite(remaining_torque) || remaining_torque*direction()<0 ||
        std::abs(remaining_torque)>std::abs(threshold)+1e-6) return false;
    // Confirmation has already withdrawn some/all torque. Do NOT reapply
    // the saved onset threshold when handing off to the batch sequencer.
    thresholds.push_back(threshold); identified.push_back(1); release_torque=remaining_torque; phase=Phase::Releasing; return true;
  }
  bool unidentified(double remaining_torque) {
    if (phase!=Phase::Probing || index()>=36 || !std::isfinite(remaining_torque) ||
        remaining_torque*direction()<0 || std::abs(remaining_torque)>FrictionProbe::limit_nm+1e-6) return false;
    // Zero is a placeholder, NEVER a measured friction value; consult identified.
    thresholds.push_back(0); identified.push_back(0);
    release_torque=remaining_torque; phase=Phase::Releasing; return true;
  }
  double withdraw(double dt) {
    if (phase!=Phase::Releasing || !std::isfinite(dt) || dt<=0 || dt>0.1) return release_torque;
    release_torque=std::copysign(std::max(0.0,std::abs(release_torque)-10.0*dt),release_torque);
    if (release_torque==0) phase=Phase::Settling;
    return release_torque;
  }
  bool settled() {
    if (phase!=Phase::Settling) return false;
    phase=index()==36?Phase::Idle:Phase::Probing; return true;
  }
  void cancel() { phase=Phase::Idle; release_torque=0; }
 private:
  std::vector<double> origin_;
};
