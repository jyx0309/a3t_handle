#include "control/mit_handle_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace {
bool allFinite(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}
}  // namespace

void MitHandleController::setJointGains(std::vector<double> kp, std::vector<double> kd) {
  joint_kp_ = std::move(kp);
  joint_kd_ = std::move(kd);
}

void MitHandleController::setJointCenterAssist(std::vector<double> center, std::vector<double> kp,
                                                std::vector<double> kd, double limit_nm, double activation_rad) {
  joint_center_ = std::move(center);
  joint_assist_kp_ = std::move(kp);
  joint_assist_kd_ = std::move(kd);
  joint_assist_limit_nm_ = limit_nm;
  joint_assist_activation_rad_ = activation_rad;
}

void MitHandleController::setStaticFrictionCompensation(std::vector<double> positive_nm,
                                                         std::vector<double> negative_nm,
                                                         double activation_nm, double velocity_rad_s) {
  static_friction_positive_ = std::move(positive_nm);
  static_friction_negative_ = std::move(negative_nm);
  static_friction_activation_nm_ = activation_nm;
  static_friction_velocity_rad_s_ = velocity_rad_s;
}

void MitHandleController::reset() {
  previous_torque_.clear();
  has_previous_ = false;
}

MitCommand MitHandleController::compute(const HandleOutput& handle, const std::vector<double>& joint_position,
                                        const std::vector<double>& joint_velocity, const std::vector<double>& gravity,
                                        const std::vector<double>& jacobian, int rows, int cols,
                                        double interaction_scale, const std::vector<double>& diagnostic_torque, double diagnostic_limit_nm) {
  MitCommand command;
  if (!std::isfinite(interaction_scale) || interaction_scale < 0.0 || interaction_scale > 1.0)
    return command;
  const int dof = static_cast<int>(joint_position.size());
  if (!diagnostic_torque.empty() && (diagnostic_torque.size()!=joint_position.size() || !allFinite(diagnostic_torque))) return command;
  const bool use_per_joint_gains = !joint_kp_.empty() || !joint_kd_.empty();
  const bool use_static_friction = !static_friction_positive_.empty() || !static_friction_negative_.empty();
  if (has_previous_ && previous_torque_.size() != joint_position.size()) return command;
  if (!handle.valid || dof == 0 || static_cast<int>(joint_velocity.size()) != dof ||
      static_cast<int>(gravity.size()) != dof || static_cast<int>(jacobian.size()) != rows * cols ||
      !(jacobian_layout_ == JacobianLayout::SpatialRows ? (rows == 6 && cols == dof)
                                                       : (rows == dof && cols == 6)) ||
      !allFinite(joint_position) || !allFinite(joint_velocity) || !allFinite(gravity) ||
      !allFinite(jacobian) || !std::isfinite(handle.dt) ||
      !std::all_of(handle.virtual_wrench.begin(), handle.virtual_wrench.end(),
                   [](double value) { return std::isfinite(value); }) ||
      !std::isfinite(safety_.control_period_s) || safety_.control_period_s <= 0.0 ||
      !std::isfinite(safety_.joint_kp) || !std::isfinite(safety_.joint_kd) ||
      safety_.joint_kp < 0.0 || safety_.joint_kd < 0.0 ||
      (use_per_joint_gains && (static_cast<int>(joint_kp_.size()) != dof ||
                               static_cast<int>(joint_kd_.size()) != dof ||
                               !allFinite(joint_kp_) || !allFinite(joint_kd_) ||
                               std::any_of(joint_kp_.begin(), joint_kp_.end(), [](double value) { return value < 0.0; }) ||
                               std::any_of(joint_kd_.begin(), joint_kd_.end(), [](double value) { return value < 0.0; })))) {
    return command;
  }
  if (use_static_friction &&
      (static_cast<int>(static_friction_positive_.size()) != dof ||
       static_cast<int>(static_friction_negative_.size()) != dof ||
       !allFinite(static_friction_positive_) || !allFinite(static_friction_negative_) ||
       std::any_of(static_friction_positive_.begin(), static_friction_positive_.end(), [](double value) { return value < 0.0; }) ||
       std::any_of(static_friction_negative_.begin(), static_friction_negative_.end(), [](double value) { return value < 0.0; }) ||
       !std::isfinite(static_friction_activation_nm_) || static_friction_activation_nm_ <= 0.0 ||
       !std::isfinite(static_friction_velocity_rad_s_) || static_friction_velocity_rad_s_ <= 0.0)) return command;

  command.position = joint_position;
  command.velocity.assign(dof, 0.0);
  command.kp = use_per_joint_gains ? joint_kp_ : std::vector<double>(dof, safety_.joint_kp);
  command.kd = use_per_joint_gains ? joint_kd_ : std::vector<double>(dof, safety_.joint_kd);
  // Explicit wire gains: scale=0 must NOT select the vendor-gain fallback.
  for (auto& value : command.kp) value *= interaction_scale;
  for (auto& value : command.kd) value *= interaction_scale;
  command.torque.resize(dof, 0.0);
  command.gravity_torque.resize(dof, 0.0);
  command.cartesian_torque.resize(dof, 0.0);
  command.joint_assist_torque.resize(dof, 0.0);
  command.static_friction_torque.resize(dof, 0.0);
  command.interaction_torque.resize(dof, 0.0);
  command.torque_saturated.resize(dof, false);
  if (!has_previous_) {
    // Entering MIT with zero feed-forward leaves gravity uncompensated for one
    // or more cycles, so seed the first frame from the measured gravity vector.
    previous_torque_.resize(dof, 0.0);
    for (int joint = 0; joint < dof; ++joint) {
      previous_torque_[joint] = gravity[joint];
      command.gravity_torque[joint] = gravity[joint];
    }
    has_previous_ = true;
    command.torque = previous_torque_;
    command.valid = true;
    return command;
  }
  for (int joint = 0; joint < dof; ++joint) {
    double cartesian_torque = 0.0;
    for (int axis = 0; axis < 6; ++axis) {
      const double j = jacobian_layout_ == JacobianLayout::JointRows
                           ? jacobian[joint * cols + axis] : jacobian[axis * cols + joint];
      cartesian_torque += j * handle.virtual_wrench[axis];
    }
    cartesian_torque *= interaction_scale;
    command.gravity_torque[joint] = gravity[joint];
    command.cartesian_torque[joint] = cartesian_torque;
    double joint_assist = 0.0;
    if (joint_center_.size() == static_cast<size_t>(dof) && joint_assist_kp_.size() == static_cast<size_t>(dof) &&
        joint_assist_kd_.size() == static_cast<size_t>(dof) && joint_assist_limit_nm_ > 0.0 &&
        joint_assist_activation_rad_ > 0.0) {
      // Smoothly bound the error of EACH joint, rather than disabling all
      // joints when any one leaves the old activation window.
      const double error = joint_center_[joint] - joint_position[joint];
      const double bounded_error = joint_assist_activation_rad_ *
          std::tanh(error / joint_assist_activation_rad_);
      joint_assist = std::clamp(joint_assist_kp_[joint] * bounded_error -
                                     joint_assist_kd_[joint] * joint_velocity[joint],
                                -joint_assist_limit_nm_, joint_assist_limit_nm_);
    }
    joint_assist *= interaction_scale;
    command.joint_assist_torque[joint] = joint_assist;
    const double return_torque = cartesian_torque + joint_assist;
    double static_friction = 0.0;
    if (use_static_friction && std::abs(return_torque) > 0.0) {
      const double force_ratio = std::clamp(std::abs(return_torque) / static_friction_activation_nm_, 0.0, 1.0);
      const double velocity_ratio = std::clamp(std::abs(joint_velocity[joint]) / static_friction_velocity_rad_s_, 0.0, 1.0);
      const double force_blend = force_ratio * force_ratio * (3.0 - 2.0 * force_ratio);
      const double velocity_blend = 1.0 - velocity_ratio * velocity_ratio * (3.0 - 2.0 * velocity_ratio);
      const double magnitude = return_torque > 0.0 ? static_friction_positive_[joint] : static_friction_negative_[joint];
      static_friction = std::copysign(magnitude * force_blend * velocity_blend * interaction_scale, return_torque);
    }
    command.static_friction_torque[joint] = static_friction;
    command.interaction_torque[joint] = cartesian_torque + joint_assist + static_friction +
        (diagnostic_torque.empty()?0.0:diagnostic_torque[joint]);
    command.torque[joint] = gravity[joint] + command.interaction_torque[joint];
  }
  previous_torque_ = command.torque;
  command.valid = true;
  return command;
}
