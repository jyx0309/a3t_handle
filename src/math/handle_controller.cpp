#include "math/handle_controller.hpp"

#include <algorithm>
#include <cmath>

bool HandleController::validPose(const std::array<double, 7>& pose) {
  if (!std::all_of(pose.begin(), pose.end(), [](double v) { return std::isfinite(v); })) return false;
  double norm = 0.0;
  for (int i = 3; i < 7; ++i) norm += pose[i] * pose[i];
  return std::abs(norm - 1.0) <= 0.01;
}

void HandleController::captureCenter(const std::array<double, 7>& pose, double timestamp_s) {
  if (!validPose(pose) || !std::isfinite(timestamp_s)) { clearCenter(); return; }
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

double HandleController::nearAssist(double displacement, double deadband, double width,
                                   double range, double amplitude, int* region) {
  const double fade_start = deadband + 0.75 * (range - deadband);
  int zone = displacement <= deadband ? 0 : displacement >= range ? 4 :
      displacement < deadband + width ? 1 : displacement <= fade_start ? 2 : 3;
  if (region) *region = zone;
  if (zone == 0 || zone == 4) return 0;
  if (zone == 1) return amplitude * smoothStep((displacement-deadband)/width);
  if (zone == 3) return amplitude * (1-smoothStep((displacement-fade_start)/(range-fade_start)));
  return amplitude;
}

double HandleController::applyDeadband(double value, double deadband) {
  if (std::abs(value) <= deadband) return 0.0;
  return std::copysign(std::abs(value) - deadband, value);
}

double HandleController::smoothStep(double value) {
  const double t = std::clamp(value, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

std::array<double, 3> HandleController::rotationError(const std::array<double, 7>& center,
                                                       const std::array<double, 7>& current) {
  // Spatial error in the base frame: q_current * inverse(q_center), xyzw.
  const double cx = -center[3], cy = -center[4], cz = -center[5], cw = center[6];
  const double x = current[3], y = current[4], z = current[5], w = current[6];
  double ex = w * cx + x * cw + y * cz - z * cy;
  double ey = w * cy - x * cz + y * cw + z * cx;
  double ez = w * cz + x * cy - y * cx + z * cw;
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
  if (!has_center_ || !validPose(pose) || !std::isfinite(timestamp_s)) return output;
  const double dt = timestamp_s - previous_time_s_;
  if (has_previous_ && (dt <= 0.0001 || dt > 0.5)) {
    has_previous_ = false;
  }
  output.dt = has_previous_ ? dt : 0.0;
  const auto rotation = rotationError(center_, pose);
  // filter_alpha is calibrated at the original 10 ms reference period.
  // Preserve its time constant when the control rate changes or jitters.
  const double reference_alpha = std::clamp(parameters_.filter_alpha, 0.0, 1.0);
  const double alpha = has_previous_ ? 1.0 - std::pow(1.0 - reference_alpha, dt / 0.01) : reference_alpha;
  for (int axis = 0; axis < 6; ++axis) {
    output.raw_error[axis] = axis < 3 ? pose[axis] - center_[axis] : rotation[axis - 3];
    const double deadband = axis < 3 ? parameters_.position_deadband_m : parameters_.rotation_deadband_rad;
    const double error = applyDeadband(output.raw_error[axis], deadband);
    output.filtered_error[axis] = has_previous_ ?
        alpha * error + (1.0 - alpha) * previous_filtered_[axis] : error;
    // Damping must also work inside the position deadband.
    filtered_velocity_[axis] = has_previous_ ?
        alpha * (output.raw_error[axis] - previous_raw_[axis]) / dt +
        (1.0 - alpha) * filtered_velocity_[axis] : 0.0;
    const double velocity = filtered_velocity_[axis];
    output.velocity[axis] = velocity;
    const double scale = axis < 3 ? parameters_.position_scale : parameters_.rotation_scale;
    const double axis_scale = axis < 3 ? parameters_.position_axis_scale[axis]
                                       : parameters_.rotation_axis_scale[axis - 3];
    const double stiffness = parameters_.independent_gains ? parameters_.cartesian_stiffness[axis] :
        axis_scale * (axis < 3 ? parameters_.position_stiffness : parameters_.rotation_stiffness);
    const double damping = parameters_.independent_gains ? parameters_.cartesian_damping[axis] :
        axis_scale * (axis < 3 ? parameters_.position_damping : parameters_.rotation_damping);
    output.command[axis] = scale * output.filtered_error[axis];
    double breakaway = 0.0;
    const double minimum = axis < 3 ? parameters_.position_return_breakaway_force_n
                                    : parameters_.rotation_return_breakaway_torque_nm;
    const double transition = axis < 3 ? parameters_.position_return_transition_m
                                       : parameters_.rotation_return_transition_rad;
    if (parameters_.near_assist_enabled[axis]) {
      breakaway = std::copysign(nearAssist(std::abs(output.raw_error[axis]), deadband,
          parameters_.near_assist_transition[axis], parameters_.near_assist_range[axis],
          parameters_.near_assist_amplitude[axis], &output.assist_region[axis]), output.raw_error[axis]);
    } else if (minimum > 0.0 && transition > 0.0) {
      output.assist_region[axis] = -1;
      const double displacement = std::abs(output.raw_error[axis]);
      const double progress = (displacement - deadband) / transition;
      // Start at zero at the deadband boundary, then smoothly reach the
      // configured Cartesian force.  This avoids a force step at the center.
      breakaway = std::copysign(minimum * smoothStep(progress), output.raw_error[axis]);
    }
    output.return_assist[axis] = -breakaway;
    output.virtual_wrench[axis] = -stiffness * output.filtered_error[axis] - damping * velocity - breakaway;
  }
  output.valid = true;
  previous_filtered_ = output.filtered_error;
  previous_raw_ = output.raw_error;
  previous_time_s_ = timestamp_s;
  has_previous_ = true;
  return output;
}
