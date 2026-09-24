#include "math/handle_controller.hpp"

#include <algorithm>
#include <cmath>

void HandleController::captureCenter(const std::array<double, 7>& pose, double timestamp_s) {
  center_ = pose;
  has_center_ = true;
  has_previous_ = false;
  previous_filtered_.fill(0.0);
  previous_time_s_ = timestamp_s;
}

void HandleController::clearCenter() {
  has_center_ = false;
  has_previous_ = false;
}

double HandleController::applyDeadband(double value, double deadband) {
  if (std::abs(value) <= deadband) return 0.0;
  return std::copysign(std::abs(value) - deadband, value);
}

std::array<double, 3> HandleController::rotationError(const std::array<double, 7>& center,
                                                       const std::array<double, 7>& current) {
  // q_err = inverse(q_center) * q_current. SDK pose quaternion order is x,y,z,w.
  const double cx = -center[3], cy = -center[4], cz = -center[5], cw = center[6];
  const double x = current[3], y = current[4], z = current[5], w = current[6];
  double ex = cw * x + cx * w + cy * z - cz * y;
  double ey = cw * y - cx * z + cy * w + cz * x;
  double ez = cw * z + cx * y - cy * x + cz * w;
  double ew = cw * w - cx * x - cy * y - cz * z;
  const double norm = std::sqrt(ex * ex + ey * ey + ez * ez + ew * ew);
  if (norm < 1e-9) return {};
  ex /= norm; ey /= norm; ez /= norm; ew /= norm;
  // Use the shortest equivalent rotation to avoid a discontinuity around pi.
  if (ew < 0.0) { ex = -ex; ey = -ey; ez = -ez; ew = -ew; }
  const double sin_half = std::sqrt(ex * ex + ey * ey + ez * ez);
  if (sin_half < 1e-9) return {};
  const double angle = 2.0 * std::atan2(sin_half, std::clamp(ew, -1.0, 1.0));
  return {angle * ex / sin_half, angle * ey / sin_half, angle * ez / sin_half};
}

HandleOutput HandleController::update(const std::array<double, 7>& pose, double timestamp_s) {
  HandleOutput output;
  if (!has_center_) return output;
  const double dt = timestamp_s - previous_time_s_;
  if (has_previous_ && (dt <= 0.0001 || dt > 0.5)) {
    has_previous_ = false;
  }
  output.dt = has_previous_ ? dt : 0.0;
  const auto rotation = rotationError(center_, pose);
  for (int axis = 0; axis < 3; ++axis) {
    output.raw_error[axis] = pose[axis] - center_[axis];
    output.raw_error[axis + 3] = rotation[axis];
    const double deadband = axis < 3 ? parameters_.position_deadband_m : parameters_.rotation_deadband_rad;
    const double error = applyDeadband(output.raw_error[axis], deadband);
    const double alpha = std::clamp(parameters_.filter_alpha, 0.0, 1.0);
    output.filtered_error[axis] = has_previous_ ?
        alpha * error + (1.0 - alpha) * previous_filtered_[axis] : error;
    const double velocity = has_previous_ ? (output.filtered_error[axis] - previous_filtered_[axis]) / dt : 0.0;
    const double scale = axis < 3 ? parameters_.position_scale : parameters_.rotation_scale;
    const double stiffness = axis < 3 ? parameters_.position_stiffness : parameters_.rotation_stiffness;
    const double damping = axis < 3 ? parameters_.position_damping : parameters_.rotation_damping;
    output.command[axis] = scale * output.filtered_error[axis];
    output.virtual_wrench[axis] = -stiffness * output.filtered_error[axis] - damping * velocity;
  }
  output.valid = true;
  previous_filtered_ = output.filtered_error;
  previous_time_s_ = timestamp_s;
  has_previous_ = true;
  return output;
}
