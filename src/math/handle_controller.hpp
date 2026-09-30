#pragma once

#include <array>

struct HandleParameters {
  std::array<bool, 6> near_assist_enabled{};
  std::array<double, 6> near_assist_amplitude{0.25,0.25,0.25,0.02,0.02,0.02};
  std::array<double, 6> near_assist_transition{0.0005,0.0005,0.0005,0.005,0.005,0.005};
  std::array<double, 6> near_assist_range{0.01,0.01,0.01,0.1,0.1,0.1};
  bool independent_gains{false};
  std::array<double, 6> cartesian_stiffness{};
  std::array<double, 6> cartesian_damping{};
  double position_scale{1.0};
  double rotation_scale{1.0};
  double position_deadband_m{0.002};
  // A small, smooth breakaway term overcomes static friction after the
  // Cartesian spring has brought the end effector close to its center.
  // It is expressed in the base-frame Cartesian axes, not joint space.
  double position_return_breakaway_force_n{0.0};
  double position_return_transition_m{0.002};
  double rotation_return_breakaway_torque_nm{0.0};
  double rotation_return_transition_rad{0.02};
  // Per-axis multipliers keep the wrench in Cartesian coordinates while
  // allowing the less capable directions to be deliberately softer.
  std::array<double, 3> position_axis_scale{1.0, 1.0, 1.0};
  double rotation_deadband_rad{0.02};
  double position_stiffness{20.0};     // N/m
  double position_damping{4.0};        // N·s/m
  double rotation_stiffness{1.0};      // N·m/rad
  double rotation_damping{0.1};        // N·m·s/rad
  std::array<double, 3> rotation_axis_scale{1.0, 1.0, 1.0};
  double filter_alpha{0.25};           // 0..1 at 10 ms; converted using actual dt
};

struct HandleOutput {
  bool valid{false};
  double dt{0.0};
  std::array<double, 6> raw_error{};       // dx,dy,dz, rotation-vector
  std::array<double, 6> filtered_error{};
  std::array<double, 6> command{};         // scaled 6-DOF output
  std::array<double, 6> virtual_wrench{};  // Fx,Fy,Fz,Tx,Ty,Tz
  std::array<double, 6> velocity{};
  std::array<double, 6> return_assist{}; // Signed requested Cartesian assistance.
  std::array<int, 6> assist_region{}; // -1 legacy, 0 deadband, 1 rise, 2 plateau, 3 fade, 4 outside.
};

// Pure math: no SDK calls and no robot command. This makes it unit-testable and
// lets the same center/error/wrench calculation be reused by the future MIT backend.
class HandleController final {
 public:
  void setParameters(const HandleParameters& parameters) { parameters_ = parameters; }
  const HandleParameters& parameters() const { return parameters_; }
  void captureCenter(const std::array<double, 7>& pose, double timestamp_s);
  void clearCenter();
  bool hasCenter() const { return has_center_; }
  static bool validPose(const std::array<double, 7>& pose);
  static double nearAssist(double displacement, double deadband, double width,
                           double range, double amplitude, int* region = nullptr);
  HandleOutput update(const std::array<double, 7>& pose, double timestamp_s);

 private:
  static std::array<double, 3> rotationError(const std::array<double, 7>& center,
                                              const std::array<double, 7>& current);
  static double applyDeadband(double value, double deadband);
  static double smoothStep(double value);

  HandleParameters parameters_;
  bool has_center_{false};
  bool has_previous_{false};
  std::array<double, 7> center_{};
  std::array<double, 6> previous_filtered_{};
  std::array<double, 6> previous_raw_{};
  std::array<double, 6> filtered_velocity_{};
  double previous_time_s_{0.0};
};
