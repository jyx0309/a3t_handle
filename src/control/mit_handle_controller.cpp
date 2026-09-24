#include "control/mit_handle_controller.hpp"

#include <algorithm>

void MitHandleController::reset() {
  previous_torque_.clear();
  has_previous_ = false;
}

MitCommand MitHandleController::compute(const HandleOutput& handle, const std::vector<double>& joint_position,
                                        const std::vector<double>& joint_velocity, const std::vector<double>& gravity,
                                        const std::vector<double>& jacobian, int rows, int cols) {
  MitCommand command;
  const int dof = static_cast<int>(joint_position.size());
  if (!handle.valid || dof == 0 || static_cast<int>(joint_velocity.size()) != dof ||
      static_cast<int>(gravity.size()) != dof || static_cast<int>(jacobian.size()) != rows * cols ||
      !((rows == dof && cols == 6) || (rows == 6 && cols == dof))) return command;

  command.position = joint_position;
  command.velocity = joint_velocity;
  command.kp.assign(dof, safety_.joint_kp);
  command.kd.assign(dof, safety_.joint_kd);
  command.torque.resize(dof, 0.0);
  for (int joint = 0; joint < dof; ++joint) {
    double cartesian_torque = 0.0;
    for (int axis = 0; axis < 6; ++axis) {
      const double j = rows == dof ? jacobian[joint * cols + axis] : jacobian[axis * cols + joint];
      cartesian_torque += j * handle.virtual_wrench[axis];
    }
    double tau = std::clamp(gravity[joint] + cartesian_torque,
                            -safety_.torque_limit_nm, safety_.torque_limit_nm);
    if (has_previous_ && handle.dt > 0.0) {
      const double maximum_delta = safety_.torque_rate_limit_nm_s * handle.dt;
      tau = std::clamp(tau, previous_torque_[joint] - maximum_delta, previous_torque_[joint] + maximum_delta);
    }
    command.torque[joint] = tau;
  }
  previous_torque_ = command.torque;
  has_previous_ = true;
  command.valid = true;
  return command;
}
