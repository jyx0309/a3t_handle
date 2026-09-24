#pragma once

#include <array>

struct HandleParameters {
  double position_scale{1.0};
  double rotation_scale{1.0};
  double position_deadband_m{0.002};
  double rotation_deadband_rad{0.02};
  double position_stiffness{20.0};     // N/m; virtual output in V0
  double position_damping{4.0};        // N·s/m; virtual output in V0
  double rotation_stiffness{1.0};      // N·m/rad; virtual output in V0
  double rotation_damping{0.1};        // N·m·s/rad; virtual output in V0
  double filter_alpha{0.25};           // 0..1, larger means less filtering
};

struct HandleOutput {
  bool valid{false};
  double dt{0.0};
  std::array<double, 6> raw_error{};       // dx,dy,dz, rotation-vector
  std::array<double, 6> filtered_error{};
  std::array<double, 6> command{};         // scaled 6-DOF output
  std::array<double, 6> virtual_wrench{};  // Fx,Fy,Fz,Tx,Ty,Tz
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
  HandleOutput update(const std::array<double, 7>& pose, double timestamp_s);

 private:
  static std::array<double, 3> rotationError(const std::array<double, 7>& center,
                                              const std::array<double, 7>& current);
  static double applyDeadband(double value, double deadband);

  HandleParameters parameters_;
  bool has_center_{false};
  bool has_previous_{false};
  std::array<double, 7> center_{};
  std::array<double, 6> previous_filtered_{};
  double previous_time_s_{0.0};
};
