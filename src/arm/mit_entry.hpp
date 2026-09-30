#pragma once
#include "arm/arm_types.hpp"

// Servo enable may be requested explicitly by the UI entry action. All other
// entry gates remain mandatory, including an unresolved low-session exit.
inline QString mitEntryBlockReason(const ArmSnapshot& s) {
  if (s.connection != ConnectionState::Connected) return "请先连接设备";
  if (s.exit_pending) return "退出观察中或退出失败锁定，请查看日志";
  if (s.mit_running || s.low_session || s.friction_batch_active) return "请先结束当前低层会话";
  if (s.safety != SafetyState::Normal || s.controller_state < 0) return "故障或急停未解除，请先检查设备";
  if (!s.log_ready) return "诊断日志不可用";
  return {};
}
