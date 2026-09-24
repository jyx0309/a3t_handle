#pragma once

#include <QJsonObject>
#include <QObject>
#include <QTimer>
#include <QStringList>
#include <memory>

#include "arm/arm_types.hpp"
#include "control/handle_mode_manager.hpp"
#include "control/mit_handle_controller.hpp"
#include "math/handle_controller.hpp"
#include "safety/safety_monitor.hpp"

namespace carm { class CArmSingleCol; }
class SessionLogger;

class ArmWorker final : public QObject {
  Q_OBJECT
 public:
  explicit ArmWorker(QJsonObject config, QObject* parent = nullptr);
  ~ArmWorker() override;

 public slots:
  void connectRobot(const QString& ip);
  void disconnectRobot();
  void setReady();
  void disableServo();
  void enterDragMode();
  void enterHandleMode();
  void captureHandleCenter();
  void applyHandleParameters(double position_scale, double rotation_scale,
                             double position_stiffness, double position_damping,
                             double rotation_stiffness, double rotation_damping);
  void startMitHandle();
  void applyDragParameters(double torque_factor, double friction_compensation_factor);
  void setSpeedLevel(double level);
  void setCollisionProtection(bool enabled, int sensitivity);
  void startTeach(const QString& name);
  void stopTeach();
  void replayTeach(const QString& name);
  void refreshTeachList();
  void exitHandleMode();
  void emergencyStop();

 signals:
  void snapshotUpdated(const ArmSnapshot& snapshot);
  void message(const QString& text);
  void teachListUpdated(const QStringList& names);

 private slots:
  void pollRobot();
  void runMitCycle();

 private:
  void publish();
  void fail(const QString& reason);
  void stopMitHandle(const QString& reason);
  bool sendDragParameters(double torque_factor, double friction_compensation_factor);
  QJsonObject config_;
  QTimer poll_timer_;
  QTimer mit_timer_;
  std::unique_ptr<carm::CArmSingleCol> robot_;
  ArmSnapshot snapshot_;
  HandleModeManager modes_;
  SafetyMonitor safety_;
  bool drag_profile_active_{false};
  HandleController handle_controller_;
  MitHandleController mit_controller_;
  bool mit_active_{false};
  int mit_failures_{0};
  std::unique_ptr<SessionLogger> logger_;
};
