#include "safety/safety_monitor.hpp"

bool SafetyMonitor::isSafeToContinue(const ArmSnapshot& snapshot, QString* reason) const {
  if (snapshot.connection != ConnectionState::Connected) {
    *reason = "Controller connection lost.";
    return false;
  }
  if (snapshot.safety != SafetyState::Normal || snapshot.controller_state < 0) {
    *reason = "Controller reports an unsafe state.";
    return false;
  }
  return true;
}
