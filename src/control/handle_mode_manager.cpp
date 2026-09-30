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
    *reason = "控制器尚未连接。";
    return false;
  }
  if (arm.servo != ServoState::Enabled) {
    *reason = "伺服未使能，请确认条件后执行复位并使能。";
    return false;
  }
  if (arm.safety != SafetyState::Normal) {
    *reason = "控制器安全状态异常。";
    return false;
  }
  mode_ = ApplicationMode::Handle;
  return true;
}
