#pragma once

#include "arm/arm_types.hpp"

class HandleModeManager {
 public:
  bool enterHandle(const ArmSnapshot& arm, QString* reason);
  void exitHandle() { mode_ = ApplicationMode::Monitor; }
  ApplicationMode mode() const { return mode_; }

 private:
  ApplicationMode mode_{ApplicationMode::Monitor};
};

QString connectionName(ConnectionState state);
QString servoName(ServoState state);
QString safetyName(SafetyState state);
QString applicationModeName(ApplicationMode mode);
