#pragma once
#include <QJsonObject>
#include <QElapsedTimer>
#include <QObject>
#include <QTimer>
#include <atomic>
#include <functional>
#include <limits>
#include <memory>
#include "arm/arm_types.hpp"
#include "control/handle_mode_manager.hpp"
#include "control/mit_handle_controller.hpp"
#include "control/mit_startup_hold.hpp"
#include "control/friction_probe.hpp"
#include "control/friction_batch.hpp"
#include "math/handle_controller.hpp"
#include "math/urdf_gravity.hpp"
#include "safety/safety_monitor.hpp"

namespace carm { class CArmSingleCol; struct RobotLowData; struct ServoStatus; }
class SessionLogger;

class ArmWorker final : public QObject {
  Q_OBJECT
 public:
  explicit ArmWorker(QJsonObject config, QObject* parent = nullptr);
  ~ArmWorker() override;
  // Thread-safe request; SDK operations remain on the worker thread.
  void requestBatchCancel() { batch_cancel_requested_.store(true); }
 public slots:
  void connectRobot(const QString& ip);
  void disconnectRobot();
  void setReady();
  void disableServo();
  void startMitHandle();
  void enableAndStartMitHandle();
  void startGravityTest();
  void startFrictionProbe(int joint, int direction);
  void startFrictionBatch();
  void exitHandleMode();
  void emergencyStop();
  void markDiagnostic(const QString& note);
  void applyTuning(QJsonObject tuning, bool save);
  void applyHandleParameters(double position_scale, double rotation_scale,
                             double position_stiffness, double position_damping,
                             double rotation_stiffness, double rotation_damping);
  // Hardware adapter only; UI must not emulate a physical deadman.
  void setDeadmanHeld(bool held);
 signals:
  void snapshotUpdated(const ArmSnapshot& snapshot);
  void message(const QString& text);
 private slots:
  void pollRobot();
  void runMitCycle();
  void advanceFrictionBatch();
 private:
  void publish();
  void abortFrictionBatch(const QString& reason);
  int traceSdk(const QString& name, const std::function<int()>& call);
  int refreshLow(carm::RobotLowData& low);
  int readServos(carm::ServoStatus& status);
  int sendMit(const MitCommand& command, carm::RobotLowData& reply);
  bool highLevelAllowed(const QString& operation);
  void fail(const QString& reason);
  void stopMitHandle(const QString& reason);
  bool enterMitLowMode(QString* reason);
  bool recoverLowCacheAndMoveToCenter(QString* reason);
  bool readConfiguredCenter(std::vector<double>* center, QString* reason) const;
  bool verifyMitJacobian(QString* reason);
  bool leaveLowMode(const QString& context);
  void registerMitFailure(const QString& reason);
  bool validateMitConfiguration(QString* reason) const;
  QJsonObject config_;
  QTimer poll_timer_, mit_timer_, batch_timer_;
  std::unique_ptr<carm::CArmSingleCol> robot_;
  ArmSnapshot snapshot_;
  HandleModeManager modes_;
  SafetyMonitor safety_;
  HandleController handle_controller_;
  MitHandleController mit_controller_;
  MitStartupHold mit_startup_hold_;
  UrdfGravity urdf_gravity_;
  UrdfGravity compensated_gravity_;
  bool gravity_compensation_active_{false};
  std::vector<double> gravity_residual_;
  bool use_urdf_gravity_{false};
  QElapsedTimer gravity_test_clock_;
  bool gravity_test_{false};
  FrictionProbe friction_probe_;
  FrictionBatch friction_batch_;
  QElapsedTimer batch_phase_clock_;
  std::atomic<bool> batch_cancel_requested_{false};
  unsigned long batch_error_baseline_{0};
  bool batch_internal_call_{false};
  QString batch_model_hash_;
  std::vector<double> batch_previous_q_, batch_settle_q_;
  double batch_previous_time_{0}, batch_stable_since_{0};
  bool mit_active_{false}, low_mode_active_{false}, low_servo_active_{false}, exit_pending_{false};
  bool minimal_exit_observing_{false}, minimal_exit_commands_ok_{false}, minimal_exit_observation_ok_{false};
  QElapsedTimer minimal_exit_clock_;
  unsigned long minimal_exit_error_baseline_{0};
  std::atomic<unsigned long> connection_epoch_{0}, sdk_error_count_{0};
  QElapsedTimer pose_received_;
  std::array<double, 7> received_pose_{};
  int mit_failures_{0}, mit_command_attempts_{0}, mit_max_failures_{3}, mit_watchdog_ms_{100};
  bool deadman_held_{false};
  QElapsedTimer mit_last_success_, mit_clock_, mode_ack_clock_, send_clock_;
  int diagnostic_frame_{0};
  std::vector<double> mit_last_accepted_position_;
  std::unique_ptr<SessionLogger> logger_;
};
