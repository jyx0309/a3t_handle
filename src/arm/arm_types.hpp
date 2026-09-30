#pragma once

#include <QMetaType>
#include <QString>

#include <array>
#include <vector>

// These are deliberately orthogonal states. A robot may be Connected +
// Enabled + Drag while the desktop application is in Handle mode.
enum class ConnectionState { Disconnected, Connected };
enum class ServoState { Disabled, Enabled };
enum class SafetyState { Normal, Fault, EmergencyStop };
enum class ApplicationMode { Monitor, Handle };

struct ArmSnapshot {
  ConnectionState connection{ConnectionState::Disconnected};
  ServoState servo{ServoState::Disabled};
  SafetyState safety{SafetyState::Normal};
  ApplicationMode application_mode{ApplicationMode::Monitor};
  int controller_state{0};
  int vendor_fsm_state{0};  // Raw ArmStatus::fsm_state; do not reinterpret vendor values here.
  bool vendor_debug_mode{false};
  bool handle_center_captured{false};
  std::vector<double> joint_position;
  std::vector<double> joint_velocity;
  std::array<double, 7> cartesian_pose{};
  std::array<double, 6> handle_command{};
  std::array<double, 6> virtual_wrench{};
  QString error;
  bool low_session{false};
  bool exit_pending{false};
  bool mit_running{false};
  bool gravity_test{false};
  bool friction_batch_active{false};
  int friction_batch_completed{0};
  bool mit_startup_holding{false};
  bool gravity_only_unlimited{false};
  bool log_ready{false};
  QString session_directory;
  QString last_sdk_call;
  int last_sdk_result{0};
  double last_sdk_ms{0};
  double last_send_gap_ms{-1};
  double first_send_delay_ms{-1};
  int mit_frames{0};
};

Q_DECLARE_METATYPE(ArmSnapshot)
