#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// One supervised direction, no automatic repositioning or parameter writes.
// Threshold is the previously acknowledged additive torque, not the next ramp step.
class FrictionProbe {
 public:
  static constexpr double limit_nm = 5.0;
  static constexpr double timeout_s = 1.0 + limit_nm / 0.2 + 0.5;
  static double rampRate(int j) { return j>=4?0.05:0.2; }
  static double axisLimit(int j) { return j>=4?0.3:limit_nm; }
  bool active{false};
  int joint{0}, direction{1};
  double applied{0}, requested{0}, threshold{0};
  double braking{0}; // Post-onset damping only; never part of the measured threshold.
  std::string outcome;
  bool start(int j, int sign, const std::vector<double>& q, double now) {
    if (active || j < 0 || j >= 6 || (sign != 1 && sign != -1) || q.size()!=6 || !std::isfinite(now)) return false;
    for (double x:q) if (!std::isfinite(x)) return false;
    origin_=q; previous_=q; start_=previous_time_=now; onset_=-1;
    stationary_since_=-1; braking=0; filtered_velocity_=0; brake_reversed_=false;
    joint=j; direction=sign; applied=requested=threshold=0; outcome.clear(); active=true;
    return true;
  }
  void cancel() { active=false; requested=braking=0; }
  void update(const std::vector<double>& q, double now) {
    if (!active) return;
    auto finish=[&](const char* reason) { outcome=reason; active=false; requested=braking=0; };
    const double dt=now-previous_time_, elapsed=now-start_;
    if (q.size()!=6 || !std::isfinite(now) || dt<=0 || dt>0.1) { finish("invalid_timing_or_state"); return; }
    for (int i=0;i<6;++i) {
      if (!std::isfinite(q[i])) { finish("invalid_state"); return; }
      // Position-derived velocity avoids a known nonzero SDK velocity floor at rest.
      if (std::abs(q[i]-previous_[i])/dt>0.3) { finish("speed_guard"); return; }
      if (std::abs(q[i]-origin_[i])>(i==joint?0.015:0.005)) { finish("displacement_guard"); return; }
      if (elapsed<=1.0 && std::abs(q[i]-origin_[i])>0.0015) { finish("baseline_drift"); return; }
    }
    const double velocity=(q[joint]-previous_[joint])/dt;
    const double position_step=std::abs(q[joint]-previous_[joint]);
    filtered_velocity_ += dt/(0.02+dt)*(velocity-filtered_velocity_);
    previous_=q; previous_time_=now;
    if (elapsed<=1.0) return;
    const double travel=direction*(q[joint]-origin_[joint]);
    if (travel < -0.003) { finish("opposite_motion"); return; }
    // Wrist motion can accelerate before reaching the old 0.003 rad trigger.
    // Stop excitation on two counts of accumulated wrist travel even when
    // separated by a pause. Requiring recent consecutive steps kept ramping
    // after genuine slow motion in the J5 recording. This is a conservative
    // motion candidate, not proof of the exact static-friction peak.
    const bool wrist_onset=joint>=4 && travel>=0.00075;
    if (onset_<0 && (travel>=0.003 || wrist_onset)) {
      onset_=now; threshold=applied; requested=applied;
    }
    if (onset_>=0) {
      // Withdraw on the FIRST observed onset, not after the confirmation wait.
      // Keep the original acknowledged threshold for identification; never
      // reverse torque during withdrawal, including for negative-direction tests.
      requested=std::copysign(std::max(0.0,std::abs(requested)-50.0*dt),requested);
      // Position-derived damping avoids the SDK velocity floor. The deadband
      // suppresses encoder quantization. Cap and slew bound this separate term.
      const double axis_cap=joint==5?0.03:(joint==4?0.04:0.8);
      const double gain=joint==5?0.2:(joint==4?0.3:4.0);
      const double cap=std::min(axis_cap,0.5*std::abs(threshold));
      // Brake only the initially observed direction. Never chase a reversal
      // with alternating torque: this is a bounded stop, not a servo loop.
      if (direction*velocity < -0.04) brake_reversed_=true;
      // On the low-inertia wrist the filter delayed the initial brake almost
      // to zero. Use raw position-derived velocity there, with the same
      // deadband, small cap and reversal latch (never increase the cap).
      // A one-count toggle is not useful braking evidence. Suppress that
      // quantization step without filtering genuine multi-count motion.
      const double brake_velocity=joint>=4?(position_step>=0.0006?velocity:0.0):filtered_velocity_;
      const double target_brake=brake_reversed_ || direction*velocity<=0 ? 0.0 :
          -direction*std::min(cap,gain*std::max(0.0,direction*brake_velocity-0.04));
      braking += std::clamp(target_brake-braking,-20.0*dt,20.0*dt);
      if (brake_reversed_ || direction*velocity<=0) braking=0;
      // Onset evidence is latched before braking. Post-brake retreat cannot
      // undo that observation, but all raw speed/displacement guards remain.
      // Do not hand off (or leave MIT for a single probe) while still coasting.
      // Require a quiet position window after all additive torque is removed.
      // Do not gate settling on a one-frame derivative: one encoder count
      // over a short cycle can exceed 0.04 rad/s while the position is stable.
      // The independent raw 0.3 rad/s guard above remains active.
      if (requested==0 && braking==0) {
        if (stationary_since_<0) {
          stationary_since_=now; stationary_min_=stationary_max_=q[joint];
        }
        stationary_min_=std::min(stationary_min_,q[joint]);
        stationary_max_=std::max(stationary_max_,q[joint]);
        if (stationary_max_-stationary_min_>0.0008) {
          stationary_since_=now; stationary_min_=stationary_max_=q[joint];
        }
        if (now-stationary_since_>=0.5) { finish("onset_candidate"); return; }
      } else stationary_since_=-1;
      if (now-onset_>=2.0) { finish("braking_timeout"); return; }
    }
    else requested=direction*std::min(axisLimit(joint),rampRate(joint)*(elapsed-1.0));
    if (onset_<0 && elapsed>=1.5+axisLimit(joint)/rampRate(joint)) finish("no_onset_below_limit");
  }
 private:
  std::vector<double> origin_, previous_;
  double start_{0}, previous_time_{0}, onset_{-1};
  double filtered_velocity_{0};
  bool brake_reversed_{false};
  double stationary_since_{-1}, stationary_min_{0}, stationary_max_{0};
};
