#pragma once

#include <vector>
#include <array>

#include "arm/arm_types.hpp"

// Compatibility-only configuration: no host-side motion restriction is
// applied from these fields.
struct MotionSafetyParameters {
  double joint_limit_margin_rad{0.0};
  double joint_velocity_fraction{0.0};
  bool enforce_joint_velocity_limit{false};
  bool require_workspace_for_mit{false};
  bool require_deadman_for_mit{false};
  bool workspace_configured{false};
  std::array<double, 3> workspace_min_m{};
  std::array<double, 3> workspace_max_m{};
};

class SafetyMonitor {
 public:
  void setMotionSafetyParameters(const MotionSafetyParameters&) {}
  void setVendorJointLimits(std::vector<double> lower, std::vector<double> upper,
                            std::vector<double> maximum_velocity);
  bool isSafeToContinue(const ArmSnapshot& snapshot, QString* reason) const;
  bool isSafeForMit(const std::vector<double>& joint_position, const std::vector<double>& joint_velocity,
                    QString* reason) const;
  bool isSafeForMit(const std::vector<double>& joint_position, const std::vector<double>& joint_velocity,
                    const std::array<double, 7>&, bool, QString* reason) const {
    return isSafeForMit(joint_position, joint_velocity, reason);
  }

 private:
  std::vector<double> joint_lower_;
  std::vector<double> joint_upper_;
  std::vector<double> joint_velocity_max_;
};
