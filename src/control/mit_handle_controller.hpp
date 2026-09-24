#pragma once

#include <vector>

#include "math/handle_controller.hpp"

struct MitSafetyParameters {
  double torque_limit_nm{5.0};
  double torque_rate_limit_nm_s{20.0};
  double joint_kp{0.0};
  double joint_kd{0.0};
};

struct MitCommand {
  bool valid{false};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;
  std::vector<double> kp;
  std::vector<double> kd;
};

class MitHandleController final {
 public:
  void setSafetyParameters(const MitSafetyParameters& parameters) { safety_ = parameters; }
  void reset();
  MitCommand compute(const HandleOutput& handle, const std::vector<double>& joint_position,
                     const std::vector<double>& joint_velocity, const std::vector<double>& gravity,
                     const std::vector<double>& jacobian, int rows, int cols);

 private:
  MitSafetyParameters safety_;
  std::vector<double> previous_torque_;
  bool has_previous_{false};
};
