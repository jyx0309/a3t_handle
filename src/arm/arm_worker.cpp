#include "arm/arm_worker.hpp"

#include <QElapsedTimer>
#include <QDateTime>
#include <stdexcept>
#include <vector>
#include <arm_control_sdk/carm_cobot.h>

#include "logging/session_logger.hpp"

ArmWorker::ArmWorker(QJsonObject config, QObject* parent) : QObject(parent), config_(std::move(config)) {
  poll_timer_.setInterval(config_.value("poll_interval_ms").toInt(100));
  connect(&poll_timer_, &QTimer::timeout, this, &ArmWorker::pollRobot);
  const auto mit = config_.value("mit").toObject();
  mit_timer_.setInterval(mit.value("period_ms").toInt(10));
  connect(&mit_timer_, &QTimer::timeout, this, &ArmWorker::runMitCycle);
  logger_ = std::make_unique<SessionLogger>(config_.value("log_directory").toString("runtime_logs"));
  const auto handle = config_.value("handle").toObject();
  HandleParameters parameters;
  parameters.position_scale = handle.value("position_scale").toDouble(1.0);
  parameters.rotation_scale = handle.value("rotation_scale").toDouble(1.0);
  parameters.position_stiffness = handle.value("position_stiffness").toDouble(20.0);
  parameters.position_damping = handle.value("position_damping").toDouble(4.0);
  parameters.rotation_stiffness = handle.value("rotation_stiffness").toDouble(1.0);
  parameters.rotation_damping = handle.value("rotation_damping").toDouble(0.1);
  handle_controller_.setParameters(parameters);
  MitSafetyParameters mit_safety;
  mit_safety.torque_limit_nm = mit.value("torque_limit_nm").toDouble(5.0);
  mit_safety.torque_rate_limit_nm_s = mit.value("torque_rate_limit_nm_s").toDouble(20.0);
  mit_safety.joint_kp = mit.value("joint_kp").toDouble(0.0);
  mit_safety.joint_kd = mit.value("joint_kd").toDouble(0.0);
  mit_controller_.setSafetyParameters(mit_safety);
}
ArmWorker::~ArmWorker() = default;

void ArmWorker::connectRobot(const QString& ip) {
  if (logger_) logger_->event("connect_requested", ip);
  disconnectRobot();
  try {
    robot_ = std::make_unique<carm::CArmSingleCol>(ip.toStdString());
    snapshot_.connection = robot_->is_connected() ? ConnectionState::Connected : ConnectionState::Disconnected;
    if (snapshot_.connection != ConnectionState::Connected) throw std::runtime_error("SDK connection was not established");
    snapshot_.safety = SafetyState::Normal;
    modes_.exitHandle();
    poll_timer_.start();
    emit message("Connected. The arm remains idle until Reset + Enable is pressed.");
    if (logger_) logger_->event("connected", ip);
  } catch (const std::exception& error) {
    robot_.reset();
    fail(QString("Connection failed: %1").arg(error.what()));
  }
  publish();
}

void ArmWorker::disconnectRobot() {
  stopMitHandle("disconnect");
  poll_timer_.stop();
  if (robot_) { robot_->set_control_mode(0); robot_->disconnect(); }
  robot_.reset();
  drag_profile_active_ = false;
  snapshot_ = {};
  modes_.exitHandle();
  if (logger_) logger_->event("disconnected");
  publish();
}

void ArmWorker::setReady() {
  if (!robot_) return fail("Connect before requesting Vendor Ready.");
  if (robot_->set_ready() != 1) return fail("Controller rejected Vendor Ready.");
  snapshot_.safety = SafetyState::Normal;
  pollRobot();
  emit message("Vendor Ready completed. Verify the returned servo and FSM state before selecting a mode.");
  if (logger_) logger_->command("set_ready");
}

void ArmWorker::disableServo() {
  if (!robot_) return fail("Connect before disabling the servo.");
  robot_->task_stop();
  robot_->set_control_mode(0);
  if (robot_->set_servo_enable(false) != 1) return fail("Controller rejected servo disable.");
  modes_.exitHandle();
  drag_profile_active_ = false;
  pollRobot();
  emit message("Servo disabled.");
  if (logger_) logger_->command("set_servo_enable", "false");
}

void ArmWorker::enterDragMode() {
  if (!robot_) return fail("Connect before entering Drag.");
  QString reason;
  if (!HandleModeManager{}.enterHandle(snapshot_, &reason)) { emit message(reason); return; }
  const auto drag = config_.value("drag").toObject();
  if (!sendDragParameters(drag.value("torque_factor").toDouble(1.0),
                          drag.value("friction_compensation_factor").toDouble(0.8)) ||
      robot_->set_control_mode(3) != 1) return fail("Controller rejected Drag mode.");
  drag_profile_active_ = true;
  modes_.exitHandle();
  snapshot_.application_mode = modes_.mode();
  emit message("Vendor Drag mode entered. This is not the application Handle mode.");
  if (logger_) logger_->command("set_control_mode", "3 (vendor drag)");
  publish();
}

void ArmWorker::enterHandleMode() {
  if (!robot_) return fail("Connect before entering Handle mode.");
  if (!handle_controller_.hasCenter()) {
    emit message("Capture the Handle center pose before entering Handle mode.");
    return;
  }
  QString reason;
  if (!modes_.enterHandle(snapshot_, &reason)) { emit message(reason); return; }
  // V0 uses vendor Drag as its physical backend. The controller calculates 6-DOF
  // command and virtual wrench but deliberately does not send MIT torque.
  const auto drag = config_.value("drag").toObject();
  if (!sendDragParameters(drag.value("torque_factor").toDouble(1.0),
                          drag.value("friction_compensation_factor").toDouble(0.8)) ||
      robot_->set_control_mode(3) != 1) {
    modes_.exitHandle();
    return fail("Controller rejected the Handle-mode Drag backend.");
  }
  drag_profile_active_ = true;
  snapshot_.application_mode = modes_.mode();
  emit message("Handle V0 entered: Drag backend with 6-DOF command and virtual-wrench logging.");
  if (logger_) logger_->command("set_control_mode", "3 (handle drag backend)");
  publish();
}

void ArmWorker::startMitHandle() {
  if (!robot_) return fail("Connect before entering MIT Handle.");
  if (!config_.value("mit").toObject().value("allow_real_mit").toBool(false)) {
    emit message("Real MIT is locked by config: set mit.allow_real_mit=true only for an approved on-site test.");
    return;
  }
  if (!handle_controller_.hasCenter() || snapshot_.servo != ServoState::Enabled ||
      snapshot_.safety != SafetyState::Normal) {
    emit message("Capture center, enable servo, and clear faults before entering MIT Handle.");
    return;
  }
  if (robot_->set_low_mode(true) != 1 || robot_->set_control_mode(2) != 1) {
    return fail("Controller rejected MIT Handle entry.");
  }
  mit_controller_.reset();
  mit_failures_ = 0;
  mit_active_ = true;
  QString mode_reason;
  modes_.enterHandle(snapshot_, &mode_reason);
  snapshot_.application_mode = modes_.mode();
  mit_timer_.start();
  if (logger_) logger_->command("mit_handle_start");
  emit message("MIT Handle started. Keep the physical emergency stop accessible.");
}

void ArmWorker::captureHandleCenter() {
  if (!robot_ || snapshot_.connection != ConnectionState::Connected) {
    emit message("Connect before capturing the Handle center pose.");
    return;
  }
  const double now_s = QDateTime::currentMSecsSinceEpoch() / 1000.0;
  handle_controller_.captureCenter(snapshot_.cartesian_pose, now_s);
  snapshot_.handle_center_captured = true;
  snapshot_.handle_command.fill(0.0);
  snapshot_.virtual_wrench.fill(0.0);
  if (logger_) logger_->event("handle_center_captured");
  emit message("Handle center captured from the current Cartesian pose.");
  publish();
}

void ArmWorker::applyHandleParameters(double position_scale, double rotation_scale,
                                      double position_stiffness, double position_damping,
                                      double rotation_stiffness, double rotation_damping) {
  if (position_scale <= 0.0 || rotation_scale <= 0.0 || position_stiffness < 0.0 ||
      position_damping < 0.0 || rotation_stiffness < 0.0 || rotation_damping < 0.0) {
    emit message("Handle scales must be positive and virtual stiffness/damping must be non-negative.");
    return;
  }
  auto parameters = handle_controller_.parameters();
  parameters.position_scale = position_scale;
  parameters.rotation_scale = rotation_scale;
  parameters.position_stiffness = position_stiffness;
  parameters.position_damping = position_damping;
  parameters.rotation_stiffness = rotation_stiffness;
  parameters.rotation_damping = rotation_damping;
  handle_controller_.setParameters(parameters);
  if (logger_) logger_->command("handle_parameters", QString("ps=%1;rs=%2;pk=%3;pd=%4;rk=%5;rd=%6")
                                  .arg(position_scale).arg(rotation_scale).arg(position_stiffness)
                                  .arg(position_damping).arg(rotation_stiffness).arg(rotation_damping));
  emit message("Applied Handle V0 parameters.");
}

void ArmWorker::applyDragParameters(double torque_factor, double friction_compensation_factor) {
  if (!robot_ || !drag_profile_active_) {
    emit message("Enter Vendor Drag or Handle mode before applying drag parameters.");
    return;
  }
  if (!sendDragParameters(torque_factor, friction_compensation_factor)) {
    return fail("Controller rejected drag parameters.");
  }
  emit message(QString("Applied drag parameters: torque factor=%1, friction compensation=%2.")
                   .arg(torque_factor, 0, 'f', 2)
                   .arg(friction_compensation_factor, 0, 'f', 2));
  if (logger_) logger_->command("set_drag_params", QString("torque=%1;friction=%2")
                                                      .arg(torque_factor, 0, 'f', 3)
                                                      .arg(friction_compensation_factor, 0, 'f', 3));
}

void ArmWorker::setSpeedLevel(double level) {
  if (!robot_) return fail("Connect before setting the speed level.");
  if (level < 0.0 || level > 10.0) {
    emit message("Speed level must be in the SDK range [0, 10].");
    return;
  }
  if (robot_->set_speed_level(level) != 1) return fail("Controller rejected speed level.");
  emit message(QString("Applied vendor speed level %1 / 10.").arg(level, 0, 'f', 1));
  if (logger_) logger_->command("set_speed_level", QString::number(level, 'f', 2));
}

void ArmWorker::setCollisionProtection(bool enabled, int sensitivity) {
  if (!robot_) return fail("Connect before configuring collision protection.");
  if (sensitivity < 0 || sensitivity > 2) {
    emit message("Collision sensitivity must be in the SDK range [0, 2].");
    return;
  }
  if (robot_->set_collision_config(enabled, sensitivity) != 1) {
    return fail("Controller rejected collision configuration.");
  }
  emit message(enabled ? QString("Collision protection enabled, sensitivity %1.").arg(sensitivity)
                       : "Collision protection disabled.");
  if (logger_) logger_->command("set_collision_config", QString("enabled=%1;sensitivity=%2")
                                                           .arg(enabled).arg(sensitivity));
}

void ArmWorker::startTeach(const QString& name) {
  if (!robot_) return fail("Connect before starting teach recording.");
  if (snapshot_.servo != ServoState::Enabled || !drag_profile_active_) {
    emit message("Enable the servo and enter Vendor Drag before recording.");
    return;
  }
  const QString clean_name = name.trimmed();
  if (clean_name.isEmpty()) {
    emit message("Enter a trajectory name before recording.");
    return;
  }
  if (robot_->trajectory_teach(true, clean_name.toStdString()) != 1) return fail("Controller rejected teach recording.");
  emit message(QString("Teach recording started: %1.").arg(clean_name));
  if (logger_) logger_->command("trajectory_teach_start", clean_name);
}

void ArmWorker::stopTeach() {
  if (!robot_) return;
  if (robot_->trajectory_teach(false, "") != 1) return fail("Controller rejected teach recording stop.");
  emit message("Teach recording stopped.");
  if (logger_) logger_->command("trajectory_teach_stop");
  refreshTeachList();
}

void ArmWorker::replayTeach(const QString& name) {
  if (!robot_) return fail("Connect before replaying a trajectory.");
  const QString clean_name = name.trimmed();
  if (clean_name.isEmpty()) {
    emit message("Select a trajectory before replay.");
    return;
  }
  if (robot_->trajectory_recorder(clean_name.toStdString(), false) != 1) return fail("Controller rejected trajectory replay.");
  emit message(QString("Trajectory replay started: %1.").arg(clean_name));
  if (logger_) logger_->command("trajectory_replay", clean_name);
}

void ArmWorker::refreshTeachList() {
  if (!robot_) return;
  std::vector<std::string> records;
  if (robot_->check_teach(records) != 1) {
    emit message("Could not read the vendor teach trajectory list.");
    return;
  }
  QStringList names;
  for (const auto& record : records) names << QString::fromStdString(record);
  emit teachListUpdated(names);
}

void ArmWorker::exitHandleMode() {
  stopMitHandle("return_to_idle");
  if (!robot_) return;
  robot_->task_stop();
  robot_->set_control_mode(0);
  modes_.exitHandle();
  drag_profile_active_ = false;
  handle_controller_.clearCenter();
  snapshot_.handle_center_captured = false;
  snapshot_.application_mode = modes_.mode();
  pollRobot();
  emit message("Returned to robot IDLE and application Monitor mode.");
  if (logger_) logger_->command("set_control_mode", "0 (idle)");
}

void ArmWorker::emergencyStop() {
  stopMitHandle("emergency_stop");
  if (robot_) robot_->emergency_stop();
  modes_.exitHandle();
  drag_profile_active_ = false;
  snapshot_.application_mode = modes_.mode();
  snapshot_.safety = SafetyState::EmergencyStop;
  emit message("Emergency stop requested. Use the physical E-stop for an immediate hazard.");
  if (logger_) logger_->command("emergency_stop");
  publish();
}

void ArmWorker::pollRobot() {
  QElapsedTimer timer;
  timer.start();
  if (!robot_) return;
  snapshot_.connection = robot_->is_connected() ? ConnectionState::Connected : ConnectionState::Disconnected;
  if (snapshot_.connection != ConnectionState::Connected) return fail("Controller connection lost.");
  const auto status = robot_->get_status();
  snapshot_.servo = status.servo_status ? ServoState::Enabled : ServoState::Disabled;
  snapshot_.controller_state = status.state;
  snapshot_.vendor_fsm_state = status.fsm_state;
  snapshot_.vendor_debug_mode = status.on_debug_mode;
  snapshot_.application_mode = modes_.mode();
  snapshot_.joint_position = robot_->get_joint_pos();
  snapshot_.cartesian_pose = robot_->get_cart_pose();
  snapshot_.handle_center_captured = handle_controller_.hasCenter();
  if (snapshot_.application_mode == ApplicationMode::Handle) {
    const auto output = handle_controller_.update(snapshot_.cartesian_pose,
                                                  QDateTime::currentMSecsSinceEpoch() / 1000.0);
    snapshot_.handle_command = output.command;
    snapshot_.virtual_wrench = output.virtual_wrench;
    if (logger_) logger_->handle(output);
  }
  QString reason;
  if (!safety_.isSafeToContinue(snapshot_, &reason)) return fail(reason);
  if (logger_) logger_->performance("state_poll", timer.nsecsElapsed() / 1'000'000.0);
  publish();
}

void ArmWorker::runMitCycle() {
  if (!mit_active_ || !robot_) return;
  QElapsedTimer timer;
  timer.start();
  carm::RobotLowData low;
  if (robot_->low_refresh(low) != 1 || low.joint_pos.empty()) {
    stopMitHandle("low_refresh failed");
    return;
  }
  std::array<double, 7> pose{};
  int tool = -1;
  carm::RobotMatrix jacobian;
  std::vector<double> mass, coriolis, gravity(low.joint_pos.size(), 0.0), zero_acc(low.joint_pos.size(), 0.0);
  if (robot_->low_get_forward_kine(low.joint_pos, pose, tool) != 1 ||
      robot_->low_get_jacobian(low.joint_pos, tool, jacobian) != 1 ||
      robot_->low_get_dynamics(low.joint_pos, low.joint_vel, zero_acc, tool, mass, coriolis, gravity) != 1) {
    stopMitHandle("MIT kinematic/dynamic query failed");
    return;
  }
  const auto handle = handle_controller_.update(pose, QDateTime::currentMSecsSinceEpoch() / 1000.0);
  const auto command = mit_controller_.compute(handle, low.joint_pos, low.joint_vel, gravity,
                                                jacobian.data, jacobian.rows, jacobian.cols);
  if (!command.valid || robot_->low_mit_command(command.position, command.velocity, command.torque,
                                                 command.kp, command.kd, low) != 1) {
    stopMitHandle("MIT command rejected");
    return;
  }
  snapshot_.handle_command = handle.command;
  snapshot_.virtual_wrench = handle.virtual_wrench;
  if (logger_) {
    logger_->handle(handle);
    logger_->performance("mit_cycle", timer.nsecsElapsed() / 1'000'000.0);
  }
}

void ArmWorker::stopMitHandle(const QString& reason) {
  if (!mit_active_) return;
  mit_timer_.stop();
  mit_active_ = false;
  if (robot_) {
    robot_->task_stop();
    robot_->set_control_mode(0);
    robot_->set_low_mode(false);
  }
  mit_controller_.reset();
  if (logger_) logger_->event("mit_handle_stop", reason);
}

void ArmWorker::fail(const QString& reason) {
  snapshot_.error = reason;
  snapshot_.safety = SafetyState::Fault;
  modes_.exitHandle();
  drag_profile_active_ = false;
  snapshot_.application_mode = modes_.mode();
  emit message(reason);
  if (logger_) logger_->event("fault", reason);
  publish();
}

bool ArmWorker::sendDragParameters(double torque_factor, double friction_compensation_factor) {
  if (torque_factor < 0.0 || torque_factor > 2.0 ||
      friction_compensation_factor < 0.0 || friction_compensation_factor > 2.0) {
    emit message("Drag parameters must both be in the SDK range [0, 2].");
    return false;
  }
  const int dof = snapshot_.joint_position.empty() ? 6 : static_cast<int>(snapshot_.joint_position.size());
  std::vector<double> torque(dof, torque_factor);
  std::vector<double> friction(dof, friction_compensation_factor);
  return robot_->set_drag_params(torque, friction) == 1;
}

void ArmWorker::publish() {
  if (logger_) logger_->state(snapshot_);
  emit snapshotUpdated(snapshot_);
}
