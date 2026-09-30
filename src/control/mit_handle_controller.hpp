#pragma once

#include <vector>

#include "math/handle_controller.hpp"

struct MitSafetyParameters {
  // Retained only for source compatibility with integrations built against
  // earlier releases.  The host no longer applies these limits.
  double torque_limit_nm{0.0};
  double interaction_torque_limit_nm{0.0};
  std::vector<double> joint_torque_limit_nm;
  double torque_rate_limit_nm_s{0.0};
  double control_period_s{0.01};
  double joint_kp{0.0};
  double joint_kd{0.0};
};

struct MitCommand {
  bool valid{false};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> torque;
  std::vector<double> gravity_torque;
  std::vector<double> cartesian_torque;
  std::vector<double> joint_assist_torque;
  std::vector<double> static_friction_torque;
  std::vector<double> interaction_torque;
  std::vector<bool> torque_saturated;
  std::vector<double> kp;
  std::vector<double> kd;
};

class MitHandleController final {
 public:
  enum class JacobianLayout { SpatialRows, JointRows };
  void setJacobianLayout(JacobianLayout layout) { jacobian_layout_ = layout; }
  void setSafetyParameters(const MitSafetyParameters& parameters) { safety_ = parameters; }
  void setJointGains(std::vector<double> kp, std::vector<double> kd);
  void setJointCenterAssist(std::vector<double> center, std::vector<double> kp,
                            std::vector<double> kd, double limit_nm, double activation_rad);
  void setStaticFrictionCompensation(std::vector<double> positive_nm,
                                     std::vector<double> negative_nm,
                                     double activation_nm, double velocity_rad_s);
  void reset();
  MitCommand compute(const HandleOutput& handle, const std::vector<double>& joint_position,
                     const std::vector<double>& joint_velocity, const std::vector<double>& gravity,
                     const std::vector<double>& jacobian, int rows, int cols,
                     double interaction_scale = 1.0,
                     const std::vector<double>& diagnostic_torque = {}, double diagnostic_limit_nm = 0.0);

 private:
  JacobianLayout jacobian_layout_{JacobianLayout::SpatialRows};
  MitSafetyParameters safety_;
  std::vector<double> joint_kp_;
  std::vector<double> joint_kd_;
  std::vector<double> previous_torque_;
  std::vector<double> joint_center_, joint_assist_kp_, joint_assist_kd_;
  std::vector<double> static_friction_positive_, static_friction_negative_;
  double joint_assist_limit_nm_{0.0};
  double joint_assist_activation_rad_{0.0};
  double static_friction_activation_nm_{0.0};
  double static_friction_velocity_rad_s_{0.0};
  bool has_previous_{false};
};
