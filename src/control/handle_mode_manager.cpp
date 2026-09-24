#include "control/handle_mode_manager.hpp"

QString connectionName(ConnectionState state) { return state == ConnectionState::Connected ? "Connected" : "Disconnected"; }
QString servoName(ServoState state) { return state == ServoState::Enabled ? "Enabled" : "Disabled"; }
QString safetyName(SafetyState state) {
  switch (state) {
    case SafetyState::Normal: return "Normal";
    case SafetyState::Fault: return "Fault";
    case SafetyState::EmergencyStop: return "Emergency stop";
  }
  return "Unknown";
}
QString applicationModeName(ApplicationMode mode) { return mode == ApplicationMode::Handle ? "Handle" : "Monitor"; }

bool HandleModeManager::enterHandle(const ArmSnapshot& arm, QString* reason) {
  if (arm.connection != ConnectionState::Connected) {
    *reason = "Controller is not connected.";
    return false;
  }
  if (arm.servo != ServoState::Enabled) {
    *reason = "Servo is not enabled. Press Reset + Enable first.";
    return false;
  }
  if (arm.safety != SafetyState::Normal) {
    *reason = "Controller safety state is not normal.";
    return false;
  }
  mode_ = ApplicationMode::Handle;
  return true;
}
