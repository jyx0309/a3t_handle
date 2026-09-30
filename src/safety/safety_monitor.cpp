#include "safety/safety_monitor.hpp"

#include <cmath>

void SafetyMonitor::setVendorJointLimits(std::vector<double> lower, std::vector<double> upper,
                                         std::vector<double> maximum_velocity) {
  joint_lower_ = std::move(lower);
  joint_upper_ = std::move(upper);
  joint_velocity_max_ = std::move(maximum_velocity);
}

bool SafetyMonitor::isSafeToContinue(const ArmSnapshot& snapshot, QString* reason) const {
  if (snapshot.connection != ConnectionState::Connected) {
    *reason = "控制器连接已断开。";
    return false;
  }
  if (snapshot.safety != SafetyState::Normal || snapshot.controller_state < 0) {
    *reason = "控制器报告异常状态。";
    return false;
  }
  return true;
}

bool SafetyMonitor::isSafeForMit(const std::vector<double>& joint_position,
                                 const std::vector<double>& joint_velocity,
                                 QString* reason) const {
  if (joint_position.empty() || joint_velocity.size() != joint_position.size()) {
    *reason = "MIT 关节状态维度无效。";
    return false;
  }
  for (size_t joint = 0; joint < joint_position.size(); ++joint) {
    if (!std::isfinite(joint_position[joint]) || !std::isfinite(joint_velocity[joint])) {
      *reason = "MIT 收到非有限数的关节状态。";
      return false;
    }
  }
  return true;
}
