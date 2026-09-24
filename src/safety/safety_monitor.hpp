#pragma once

#include "arm/arm_types.hpp"

class SafetyMonitor {
 public:
  bool isSafeToContinue(const ArmSnapshot& snapshot, QString* reason) const;
};
