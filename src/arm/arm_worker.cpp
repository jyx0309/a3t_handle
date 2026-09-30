#include "arm/arm_worker.hpp"
#include "arm/mit_entry.hpp"
#include "control/pv_trajectory.hpp"

#include <QElapsedTimer>
#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QSaveFile>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QMetaObject>
#include <QThread>
#include <QStringList>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>
#include <arm_control_sdk/carm_cobot.h>

#include "logging/session_logger.hpp"
#include "config/config_validation.hpp"
#include "config/near_assist.hpp"

namespace {
constexpr auto kVendorErrorCallbackKey = "a3t_handle";

template <typename T> QJsonArray numbers(const T& values) {
  QJsonArray out;
  for (const auto v : values) out.append(static_cast<double>(v));
  return out;
}

QJsonObject lowRecord(const carm::RobotLowData& low) {
  return {{"error_code", low.status.error_code}, {"error_msg", QString::fromStdString(low.status.error_msg)},
          {"connected", low.status.arm_connected}, {"enabled", low.status.arm_enable},
          {"healthy", low.status.arm_status}, {"mode", static_cast<int>(low.status.arm_mode)},
          {"q", numbers(low.joint_pos)}, {"dq", numbers(low.joint_vel)}, {"tau", numbers(low.joint_tau)},
          {"q_command", numbers(low.joint_cmd_pos)}, {"tau_command", numbers(low.joint_cmd_tau)},
          {"gripper_error", low.status.gripper_error_code}};
}

template <typename T>
QString joinNumericValues(const std::vector<T>& values) {
  QStringList parts;
  parts.reserve(static_cast<int>(values.size()));
  for (const auto value : values) parts.push_back(QString::number(static_cast<double>(value), 'g', 12));
  return parts.join(';');
}

}  // namespace

ArmWorker::ArmWorker(QJsonObject config, QObject* parent) : QObject(parent), config_(std::move(config)) {
  // QTimer objects must move with ArmWorker before they are started from its worker thread.
  // Giving these member timers the worker as parent makes moveToThread() transfer their affinity.
  poll_timer_.setParent(this);
  mit_timer_.setParent(this);
  batch_timer_.setParent(this);
  batch_timer_.setInterval(100);
  connect(&batch_timer_, &QTimer::timeout, this, &ArmWorker::advanceFrictionBatch);
  poll_timer_.setInterval(config_.value("poll_interval_ms").toInt(100));
  connect(&poll_timer_, &QTimer::timeout, this, &ArmWorker::pollRobot);
  const auto mit = config_.value("mit").toObject();
  mit_timer_.setInterval(mit.value("period_ms").toInt(10));
  mit_timer_.setTimerType(Qt::PreciseTimer);
  connect(&mit_timer_, &QTimer::timeout, this, &ArmWorker::runMitCycle);
  logger_ = std::make_unique<SessionLogger>(config_.value("log_directory").toString("runtime_logs"));
  logger_->diagnostic({{"type", "session_metadata"}, {"config", config_},
      {"build", QString(__DATE__ " " __TIME__)}, {"sdk_sha256", QString(SDK_BINARY_SHA256)},
      {"controller_safety_limits", "unknown; application limits are not controller limits"}});
  const auto handle = config_.value("handle").toObject();
  HandleParameters parameters;
  parameters.position_scale = handle.value("position_scale").toDouble(1.0);
  parameters.rotation_scale = handle.value("rotation_scale").toDouble(1.0);
  parameters.position_deadband_m = handle.value("position_deadband_m").toDouble(0.002);
  parameters.position_return_breakaway_force_n = handle.value("position_return_breakaway_force_n").toDouble(0.0);
  parameters.position_return_transition_m = handle.value("position_return_transition_m").toDouble(0.002);
  parameters.rotation_deadband_rad = handle.value("rotation_deadband_rad").toDouble(0.02);
  parameters.rotation_return_breakaway_torque_nm = handle.value("rotation_return_breakaway_torque_nm").toDouble(0.0);
  parameters.rotation_return_transition_rad = handle.value("rotation_return_transition_rad").toDouble(0.02);
  const auto position_axis_scale = handle.value("position_axis_scale").toArray();
  const auto rotation_axis_scale = handle.value("rotation_axis_scale").toArray();
  for (int axis = 0; axis < 3; ++axis) {
    if (axis < position_axis_scale.size())
      parameters.position_axis_scale[axis] = position_axis_scale.at(axis).toDouble(1.0);
    if (axis < rotation_axis_scale.size())
      parameters.rotation_axis_scale[axis] = rotation_axis_scale.at(axis).toDouble(1.0);
  }
  parameters.position_stiffness = handle.value("position_stiffness").toDouble(20.0);
  parameters.position_damping = handle.value("position_damping").toDouble(4.0);
  parameters.rotation_stiffness = handle.value("rotation_stiffness").toDouble(1.0);
  parameters.rotation_damping = handle.value("rotation_damping").toDouble(0.1);
  if (handle.contains("cartesian_stiffness") && handle.contains("cartesian_damping")) {
    parameters.independent_gains = true;
    for (int i = 0; i < 6; ++i) {
      parameters.cartesian_stiffness[i] = handle.value("cartesian_stiffness").toArray().at(i).toDouble();
      parameters.cartesian_damping[i] = handle.value("cartesian_damping").toArray().at(i).toDouble();
    }
  }
  loadNearAssist(handle, parameters);
  handle_controller_.setParameters(parameters);
  MitSafetyParameters mit_safety;
  mit_safety.control_period_s = mit.value("period_ms").toDouble(10.0) / 1000.0;
  mit_safety.joint_kp = mit.value("joint_kp").toDouble(0.0);
  mit_safety.joint_kd = mit.value("joint_kd").toDouble(0.0);
  mit_controller_.setSafetyParameters(mit_safety);
  mit_max_failures_ = mit.value("max_failures").toInt(3);
  mit_watchdog_ms_ = mit.value("watchdog_ms").toInt(100);
}
ArmWorker::~ArmWorker() = default;

int ArmWorker::traceSdk(const QString& name, const std::function<int()>& call) {
  if (logger_) logger_->diagnostic({{"type", "rpc_begin"}, {"call", name}});
  QElapsedTimer elapsed;
  elapsed.start();
  int result = -1;
  QString exception;
  try { result = call(); }
  catch (const std::exception& error) { exception = QString::fromUtf8(error.what()); }
  catch (...) { exception = "unknown SDK exception"; }
  snapshot_.last_sdk_call = name;
  snapshot_.last_sdk_result = result;
  snapshot_.last_sdk_ms = elapsed.nsecsElapsed() / 1e6;
  if (logger_) logger_->diagnostic({{"type", "rpc_end"}, {"call", name}, {"result", result},
      {"duration_ms", snapshot_.last_sdk_ms}, {"exception", exception}});
  return result;
}

int ArmWorker::refreshLow(carm::RobotLowData& low) {
  const int result = traceSdk("low_refresh", [&] { return robot_->low_refresh(low); });
  if (logger_) logger_->diagnostic({{"type", "low_state"}, {"result", result},
      {"data_valid", result == 1}, {"state", lowRecord(low)}});
  return result;
}

int ArmWorker::sendMit(const MitCommand& command, carm::RobotLowData& reply) {
  // Capture complete host-side input. This is not a wire capture or proof of execution.
  const int sequence = ++diagnostic_frame_;
  if (logger_) logger_->diagnostic({{"type", "mit_input"}, {"frame", sequence},
      {"q", numbers(command.position)}, {"dq", numbers(command.velocity)}, {"tau_ff", numbers(command.torque)},
      {"kp", numbers(command.kp)}, {"kd", numbers(command.kd)}});
  const double gap = send_clock_.isValid() ? send_clock_.nsecsElapsed() / 1e6 : -1;
  send_clock_.start();
  const double since_mode = mode_ack_clock_.isValid() ? mode_ack_clock_.nsecsElapsed() / 1e6 : -1;
  if (sequence == 1) snapshot_.first_send_delay_ms = since_mode;
  snapshot_.last_send_gap_ms = gap;
  snapshot_.mit_frames = sequence;
  QElapsedTimer duration;
  duration.start();
  // Use a new output object: failed calls must not appear to return the input refresh state.
  reply = carm::RobotLowData{};
  int result = -1;
  QString exception;
  try { result = robot_->low_mit_command(command.position, command.velocity, command.torque,
                                        command.kp, command.kd, reply); }
  catch (const std::exception& e) { exception = QString::fromUtf8(e.what()); }
  catch (...) { exception = "unknown SDK exception"; }
  snapshot_.last_sdk_call = "low_mit_command";
  snapshot_.last_sdk_result = result;
  snapshot_.last_sdk_ms = duration.nsecsElapsed() / 1e6;
  if (logger_) logger_->diagnostic({{"type", "mit_result"}, {"frame", sequence}, {"result", result},
      {"duration_ms", snapshot_.last_sdk_ms}, {"send_gap_ms", gap}, {"since_mode_ack_ms", since_mode},
      {"reply_valid", result == 1}, {"reply", lowRecord(reply)}, {"exception", exception}});
  return result;
}

int ArmWorker::readServos(carm::ServoStatus& status) {
  const int result = traceSdk("low_get_servo_status", [&] { return robot_->low_get_servo_status(status); });
  QJsonArray errors;
  for (const auto& text : status.motorErrorMsg) errors.append(QString::fromStdString(text));
  if (logger_) logger_->diagnostic({{"type", "servo_state"}, {"result", result},
      {"data_valid", result == 1}, {"fsm", numbers(status.fsmMode)},
      {"enabled", numbers(status.isServoEnable)}, {"connected", numbers(status.isConnected)},
      {"error_codes", numbers(status.motorErrorCode)}, {"error_messages", errors}});
  return result;
}

void ArmWorker::markDiagnostic(const QString& note) {
  if (logger_) logger_->diagnostic({{"type", "operator_note"}, {"note", note.left(1000)}});
  emit message("现场备注已记录：" + note.left(1000));
  publish();
}

bool ArmWorker::highLevelAllowed(const QString& operation) {
  if (!logger_ || !logger_->isReady()) {
    emit message(operation + "：诊断日志不可用。");
    return false;
  }
  if (!low_mode_active_ && !exit_pending_) return true;
  emit message(operation + "：请等待低层会话退出并完成验收。");
  return false;
}

void ArmWorker::connectRobot(const QString& ip) {
  if (!logger_ || !logger_->isReady()) { emit message("会话日志不可用，已拒绝连接。"); return; }
  if (logger_) logger_->event("connect_requested", ip);
  disconnectRobot();
  try {
    robot_ = std::make_unique<carm::CArmSingleCol>(ip.toStdString());
    snapshot_.connection = robot_->is_connected() ? ConnectionState::Connected : ConnectionState::Disconnected;
    if (snapshot_.connection != ConnectionState::Connected) throw std::runtime_error("SDK connection was not established");
    const auto epoch = connection_epoch_.load();
    robot_->register_pose_cbk([this, epoch](double, std::array<double, 7> pose) {
      QElapsedTimer arrival;
      arrival.start();
      QMetaObject::invokeMethod(this, [this, epoch, pose, arrival] {
        if (epoch != connection_epoch_.load()) return;
        received_pose_ = pose;
        pose_received_ = arrival;
      }, Qt::QueuedConnection);
    });
    robot_->register_error_cbk(kVendorErrorCallbackKey, [this, epoch](int code, const std::string text) {
      if (epoch != connection_epoch_.load()) return;
      ++sdk_error_count_;
      const auto received = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
      const QString message_text = QString("厂商 SDK 错误 %1：%2")
                                       .arg(code)
                                       .arg(QString::fromStdString(text));
      QMetaObject::invokeMethod(this, [this, epoch, code, message_text, received] {
        if (epoch != connection_epoch_.load()) return;
        if (logger_) logger_->event("vendor_error_callback",
                                    QString("received=%1;code=%2;message=%3").arg(received).arg(code).arg(message_text));
        if (mit_active_ || exit_pending_ || code == 202) fail(message_text);
        else emit message(message_text);
      }, Qt::QueuedConnection);
    });
    snapshot_.safety = SafetyState::Normal;
    const auto arm_config = robot_->get_config();
    if (logger_) logger_->diagnostic({{"type", "controller_metadata"},
        {"reported_version", QString::fromStdString(robot_->get_version())},
        {"joint_lower", numbers(arm_config.limit_lower)}, {"joint_upper", numbers(arm_config.limit_upper)},
        {"joint_velocity_max", numbers(arm_config.joint_vel)}});
    safety_.setVendorJointLimits(arm_config.limit_lower, arm_config.limit_upper, arm_config.joint_vel);
    modes_.exitHandle();
    poll_timer_.start();
    emit message("连接已建立。请确认设备状态，复位并使能会影响真实机械臂。");
    if (logger_) logger_->event("connected", ip);
  } catch (const std::exception& error) {
    robot_.reset();
    fail(QString("连接失败：%1").arg(error.what()));
  }
  publish();
}

void ArmWorker::disconnectRobot() {
  ++connection_epoch_;
  stopMitHandle("disconnect");
  poll_timer_.stop();
  if (robot_) {
    robot_->release_error_cbk(kVendorErrorCallbackKey);
    robot_->release_pose_cbk();
    if (low_mode_active_) leaveLowMode("disconnect_retry");
    robot_->disconnect();
  }
  robot_.reset();
  low_mode_active_ = false;
  low_servo_active_ = false;
  exit_pending_ = false;
  minimal_exit_observing_ = false;
  pose_received_.invalidate();
  handle_controller_.clearCenter();
  deadman_held_ = false;
  snapshot_ = {};
  modes_.exitHandle();
  if (logger_) logger_->event("disconnected");
  publish();
}

void ArmWorker::setReady() {
  if (!highLevelAllowed("Ready")) return;
  if (low_mode_active_) { emit message("Exit the low-level session before Ready."); return; }
  if (!robot_) return fail("Connect before requesting Vendor Ready.");
  if (robot_->set_ready() != 1) return fail("Controller rejected Vendor Ready.");
  snapshot_.safety = SafetyState::Normal;
  snapshot_.error.clear();
  pollRobot();
  emit message("复位并使能请求已返回，请确认伺服与控制器模式后再选择操作。");
  if (logger_) logger_->command("set_ready");
}

void ArmWorker::disableServo() {
  if (!robot_) return fail("Connect before disabling the servo.");
  stopMitHandle("servo_disable");
  // A broken log must never suppress an explicit stop request. Low-session
  // cleanup has already requested disable; do not mix transports if unresolved.
  if (low_mode_active_ || exit_pending_) return;
  traceSdk("task_stop", [&] { return robot_->task_stop(); });
  traceSdk("set_control_mode", [&] { return robot_->set_control_mode(0); });
  if (traceSdk("set_servo_enable", [&] { return robot_->set_servo_enable(false); }) != 1) return fail("Controller rejected servo disable.");
  modes_.exitHandle();
  pollRobot();
  emit message("下使能请求已发送，请核对状态。");
  if (logger_) logger_->command("set_servo_enable", "false");
}

void ArmWorker::enableAndStartMitHandle() {
  // UI-only explicit confirmation; do not change the remote start contract.
  if (!robot_ || !highLevelAllowed("进入手柄")) return;
  pollRobot();
  const auto blocked = mitEntryBlockReason(snapshot_);
  if (!blocked.isEmpty()) { emit message(blocked); return; }
  if (!config_.value("mit").toObject().value("allow_real_mit").toBool(false)) {
    emit message("连续 MIT 已被配置锁定。"); return;
  }
  if (snapshot_.servo != ServoState::Enabled) {
    const auto baseline = sdk_error_count_.load();
    // Enable only: never implicitly reset a controller fault to enter MIT.
    if (traceSdk("mit_entry_servo_enable", [&] { return robot_->set_servo_enable(true); }) != 1)
      return fail("进入手柄前使能失败，未启动 MIT。");
    pollRobot();
    if (sdk_error_count_.load() != baseline || !mitEntryBlockReason(snapshot_).isEmpty() ||
        snapshot_.servo != ServoState::Enabled) {
      emit message("尚未确认健康使能反馈，未启动手柄；请检查状态后重新点击，不会自动重试。");
      return;
    }
  }
  startMitHandle();
}

void ArmWorker::startMitHandle() {
  if (friction_batch_.active() && !batch_internal_call_) { emit message("批次运行中，先停止整批。"); return; }
  if (!highLevelAllowed("MIT entry")) return;
  if (mit_active_ || low_mode_active_) {
    emit message("请先退出当前低层会话，再启动 MIT 手柄。");
    return;
  }
  if (!robot_) return fail("Connect before entering MIT Handle.");
  if (!config_.value("mit").toObject().value("allow_real_mit").toBool(false)) {
    emit message("连续 MIT 已被配置锁定，仅在经确认的现场试验中开放。");
    return;
  }
  if (snapshot_.servo != ServoState::Enabled || snapshot_.safety != SafetyState::Normal) {
    emit message("进入 MIT 前须确认使能并排除故障。");
    return;
  }
  // The controller retains a Low-Mode position cache across high-level moves.
  // Reconcile the real arm to that cache first, then use PV interpolation to
  // reach the configured hand centre before selecting MIT mode.
  std::vector<double> startup_center;
  QString config_reason;
  if (!validateMitConfiguration(&config_reason)) { emit message(config_reason); return; }
  const auto gravity_config=config_.value("gravity").toObject();
  use_urdf_gravity_=gravity_config.value("source").toString("sdk")=="urdf";
  gravity_compensation_active_=false;
  if (use_urdf_gravity_) {
    const auto directory=QFileInfo(config_.value("configuration_file").toString()).absoluteDir();
    const auto path=directory.absoluteFilePath(gravity_config.value("urdf_path").toString());
    if (!urdf_gravity_.load(path, &config_reason)) {
      emit message("URDF 重力模型加载失败，未启动运动："+config_reason); return;
    }
    logger_->diagnostic({{"type","gravity_model"},{"source","urdf"},{"path",path},
        {"sha256",urdf_gravity_.sha256()},{"gravity_base_z",-9.81},{"finger_position_m",0.0}});
    const auto compensation=gravity_config.value("compensation").toObject();
    if (compensation.value("enabled").toBool(false)) {
      std::vector<double> delta;
      gravity_residual_.clear();
      for (const auto v:compensation.value("parameter_delta").toArray()) {
        if (!v.isDouble()) { emit message("Invalid gravity compensation parameter"); return; }
        delta.push_back(v.toDouble());
      }
      for (const auto v:compensation.value("residual_nm").toArray()) {
        if (!v.isDouble() || !std::isfinite(v.toDouble()) || std::abs(v.toDouble())>0.5) {
          emit message("Invalid gravity compensation residual"); return;
        }
        gravity_residual_.push_back(v.toDouble());
      }
      if (gravity_residual_.size()!=6 || compensation.value("base_sha256").toString()!=urdf_gravity_.sha256()) {
        emit message("Gravity compensation residual size/model hash mismatch"); return;
      }
      compensated_gravity_=urdf_gravity_;
      if (!compensated_gravity_.applyGravityFit(delta,&config_reason)) { emit message(config_reason); return; }
      gravity_compensation_active_=true;
      logger_->diagnostic({{"type","gravity_compensation_model"},{"config",compensation},
          {"applied",true},{"mode","full"}});
      emit message("重力补偿：已启用完整辨识模型（物理参数与残差，含 J2）。");
    }
  }
  emit message(use_urdf_gravity_ ? "重力来源：本地 URDF（含模型夹爪，不含额外摄像头）；SDK 仅作对照。" : "重力来源：SDK");
  if (!readConfiguredCenter(&startup_center, &config_reason)) return fail(config_reason);
  gravity_test_ = false;
  snapshot_.gravity_test = false;
  snapshot_.gravity_only_unlimited = false;
  if (!recoverLowCacheAndMoveToCenter(&config_reason)) return fail(config_reason);
  mit_startup_hold_.reset(startup_center);
  if (!enterMitLowMode(&config_reason)) return fail(config_reason);
  mit_controller_.reset();
  mit_last_accepted_position_.clear();
  mit_clock_.start();
  mit_failures_ = 0;
  mit_command_attempts_ = 0;
  mit_last_success_.start();
  mit_active_ = true;
  QString mode_reason;
  modes_.enterHandle(snapshot_, &mode_reason);
  snapshot_.application_mode = modes_.mode();
  // Do not leave the controller waiting for the first 10 ms timer tick.
  runMitCycle();
  if (!mit_active_) return;
  mit_timer_.start();
  if (logger_) logger_->command("mit_handle_start");
  emit message("MIT 已启动中心保持，稳定后平滑释放到手柄模式；启动期间请勿拖动。");
}

bool ArmWorker::readConfiguredCenter(std::vector<double>* center, QString* reason) const {
  const auto configured = config_.value("mit").toObject().value("center_joint_position_rad").toArray();
  if (configured.empty()) { *reason = "MIT 配置缺少 center_joint_position_rad。"; return false; }
  center->clear();
  center->reserve(configured.size());
  for (const auto value : configured) {
    const double joint = value.toDouble(std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(joint)) { *reason = "MIT 中心关节角包含无效值。"; return false; }
    center->push_back(joint);
  }
  return true;
}

bool ArmWorker::recoverLowCacheAndMoveToCenter(QString* reason) {
  constexpr double kEntryGapRad = 0.09;
  constexpr double kArrivalRad = 0.025;
  constexpr double kSettledVelocityRadS = 0.03;
  std::vector<double> center;
  if (!readConfiguredCenter(&center, reason)) return false;
  const auto mit_config = config_.value("mit").toObject();
  const auto assist_kp_json = mit_config.value("joint_center_assist_kp_nm_rad").toArray();
  const auto assist_kd_json = mit_config.value("joint_center_assist_kd_nm_s_rad").toArray();
  std::vector<double> assist_kp, assist_kd;
  if (assist_kp_json.size() != static_cast<int>(center.size()) ||
      assist_kd_json.size() != static_cast<int>(center.size())) {
    *reason = "关节中心辅助参数自由度不匹配。"; return false;
  }
  for (size_t i = 0; i < center.size(); ++i) {
    const double kp = assist_kp_json[static_cast<int>(i)].toDouble(std::numeric_limits<double>::quiet_NaN());
    const double kd = assist_kd_json[static_cast<int>(i)].toDouble(std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(kp) || !std::isfinite(kd) || kp < 0.0 || kd < 0.0) {
      *reason = "关节中心辅助参数无效。"; return false;
    }
    assist_kp.push_back(kp); assist_kd.push_back(kd);
  }
  mit_controller_.setJointCenterAssist(center, assist_kp, assist_kd,
      mit_config.value("joint_center_assist_limit_nm").toDouble(0.0),
      mit_config.value("joint_center_assist_activation_rad").toDouble(0.0));
  std::vector<double> static_positive, static_negative;
  for (const auto value : mit_config.value("static_friction_positive_nm").toArray())
    static_positive.push_back(value.toDouble(std::numeric_limits<double>::quiet_NaN()));
  for (const auto value : mit_config.value("static_friction_negative_nm").toArray())
    static_negative.push_back(value.toDouble(std::numeric_limits<double>::quiet_NaN()));
  mit_controller_.setStaticFrictionCompensation(std::move(static_positive), std::move(static_negative),
      mit_config.value("static_friction_activation_nm").toDouble(0.0),
      mit_config.value("static_friction_velocity_rad_s").toDouble(0.0));

  emit message("MIT 回接：读取低层命令缓存（未启用 Low servo）。");
  const int low_on = traceSdk("mit_cache_read_set_low_mode", [&] { return robot_->set_low_mode(true); });
  if (low_on != 1) { *reason = "无法进入 Low Mode 读取缓存。"; return false; }
  low_mode_active_ = true;
  QThread::msleep(10);
  carm::RobotLowData cached;
  const int read = refreshLow(cached);
  if (read != 1 || cached.joint_pos.size() != center.size() ||
      cached.joint_cmd_pos.size() != center.size()) {
    leaveLowMode("mit_cache_read_failed");
    *reason = "低层缓存读取失败或关节自由度不匹配。";
    return false;
  }
  if (logger_) logger_->event("mit_cache_read", QString("actual=%1;cached=%2")
      .arg(joinNumericValues(cached.joint_pos)).arg(joinNumericValues(cached.joint_cmd_pos)));
  // No low-level servo/command has been enabled at this point.  Do not call
  // leaveLowMode() here: that routine deliberately disables the high-level
  // servo, causing an avoidable gravity drop before the recovery motion.
  const int cache_low_off = traceSdk("mit_cache_read_set_low_mode_false", [&] { return robot_->set_low_mode(false); });
  if (cache_low_off != 1) { *reason = "读取低层缓存后无法退出 Low Mode。"; return false; }
  low_mode_active_ = false;
  QThread::msleep(10);
  if (friction_batch_.active()) {
    for (size_t i=0;i<center.size();++i) {
      if (batch_cancel_requested_.load() || sdk_error_count_.load()!=batch_error_baseline_ ||
          !std::isfinite(cached.joint_cmd_pos[i]) || std::abs(cached.joint_cmd_pos[i]-center[i])>0.05) {
        *reason="批次取消/错误，或缓存离开中心邻域。"; return false;
      }
    }
  }
  if (traceSdk("mit_cache_read_task_stop", [&] { return robot_->task_stop(); }) != 1 ||
      traceSdk("mit_cache_read_idle", [&] { return robot_->set_control_mode(0); }) != 1) {
    *reason = "读取低层缓存后无法安全返回高层空闲。"; return false;
  }
  if (logger_) logger_->event("mit_cache_read_exit", "low_transport_off;high_servo_kept_enabled");

  const double recovery_speed = config_.value("mit").toObject().value("cache_recovery_speed_level").toDouble(5.0);
  emit message(QString("MIT 回接：高层以速度等级 %1 / 10 运动到低层缓存姿态。").arg(recovery_speed, 0, 'f', 1));
  if (traceSdk("mit_cache_recovery_ready", [&] { return robot_->set_ready(); }) != 1 ||
      traceSdk("mit_cache_recovery_speed", [&] { return robot_->set_speed_level(recovery_speed); }) != 1) {
    *reason = "无法高层慢速移动到低层缓存姿态。"; return false;
  }
  const auto recovery_start = robot_->get_joint_pos();
  if (recovery_start.size() != center.size()) { *reason = "无法读取高层回接起点。"; return false; }
  double largest_move = 0.0;
  for (size_t i = 0; i < center.size(); ++i)
    largest_move = std::max(largest_move, std::abs(cached.joint_cmd_pos[i] - recovery_start[i]));
  const double duration_s = std::max(2.0, largest_move / 0.10);
  if (logger_) logger_->event("mit_cache_recovery_trajectory",
      QString("vendor_default_move;requested_duration_s=%1;max_joint_delta_rad=%2")
          .arg(duration_s, 0, 'f', 3).arg(largest_move, 0, 'f', 6));
  if (friction_batch_.active() && (batch_cancel_requested_.load() || largest_move>0.01)) {
    *reason="批次缓存偏差过大或已取消，不自动进行高层缓存运动。"; return false;
  }
  if (!friction_batch_.active() && traceSdk("mit_cache_recovery_move", [&] {
        return robot_->move_joint(cached.joint_cmd_pos, -1, true);
      }) != 1) {
    *reason = "高层定时平滑缓存回接被拒绝。"; return false;
  }
  const auto reached_cache = robot_->get_joint_pos();
  double cache_gap = 0.0;
  for (size_t i = 0; i < reached_cache.size(); ++i) cache_gap = std::max(cache_gap, std::abs(reached_cache[i] - cached.joint_cmd_pos[i]));
  if (reached_cache.size() != center.size() || cache_gap > kArrivalRad) {
    *reason = QString("高层未到达低层缓存姿态（最大差 %1 rad）。").arg(cache_gap, 0, 'f', 4); return false;
  }
  traceSdk("mit_cache_recovery_stop", [&] { return robot_->task_stop(); });

  emit message("MIT 回接：进入 Low PV，向配置中心点小步过渡。");
  if (traceSdk("mit_pv_set_low_mode", [&] { return robot_->set_low_mode(true); }) != 1) {
    *reason = "无法重新进入 Low Mode。"; return false;
  }
  low_mode_active_ = true;
  QThread::msleep(10);
  carm::RobotLowData low;
  if (refreshLow(low) != 1 || low.joint_pos.size() != center.size() || low.joint_cmd_pos.size() != center.size() ||
      !std::all_of(low.joint_pos.begin(), low.joint_pos.end(), [](double v) { return std::isfinite(v); }) ||
      !std::all_of(low.joint_cmd_pos.begin(), low.joint_cmd_pos.end(), [](double v) { return std::isfinite(v); })) {
    leaveLowMode("mit_pv_initial_refresh_failed"); *reason = "PV 过渡前无法读取低层状态。"; return false;
  }
  double entry_gap = 0.0;
  for (size_t i = 0; i < center.size(); ++i) entry_gap = std::max(entry_gap, std::abs(low.joint_pos[i] - low.joint_cmd_pos[i]));
  if (entry_gap > kEntryGapRad) { leaveLowMode("mit_pv_entry_gap"); *reason = QString("缓存回接后仍相差 %1 rad。").arg(entry_gap); return false; }
  if (traceSdk("mit_pv_low_servo_enable", [&] { return robot_->low_set_servo_enable(true); }) != 1 ||
      traceSdk("mit_pv_robot_mode", [&] { return robot_->low_set_robot_mode(1); }) != 1) {
    leaveLowMode("mit_pv_enable_failed"); *reason = "无法启用 Low PV 模式。"; return false;
  }
  low_servo_active_ = true;
  const PvTrajectory trajectory(low.joint_pos, center);
  const double pv_duration_s = trajectory.duration_s;
  size_t steps = 0;
  QElapsedTimer pv_clock;
  pv_clock.start();
  qint64 deadline_ns = 0;
  double previous_send_ms = -1.0;
  const auto pv_error_baseline = sdk_error_count_.load();
  auto wait_next = [&] {
    deadline_ns = PvTrajectory::nextDeadlineNs(deadline_ns, pv_clock.nsecsElapsed());
    const auto remaining = deadline_ns - pv_clock.nsecsElapsed();
    if (remaining > 0) QThread::usleep(static_cast<unsigned long>((remaining + 999) / 1000));
  };
  auto send_pv = [&](const std::vector<double>& target, const std::vector<double>& velocity,
                     bool holding, carm::RobotLowData& reply) {
    if (!logger_->isReady() || sdk_error_count_.load() != pv_error_baseline) return false;
    const double sent_ms = pv_clock.nsecsElapsed() / 1e6;
    const int result = traceSdk(holding ? "mit_pv_center_hold" : "low_pv_command",
        [&] { return robot_->low_pv_command(target, velocity, reply); });
    const double rpc_ms = pv_clock.nsecsElapsed()/1e6 - sent_ms;
    const bool valid = result == 1 && reply.status.error_code == 0 &&
        reply.status.arm_connected && reply.status.arm_enable && reply.status.arm_status &&
        reply.joint_pos.size() == center.size() && reply.joint_vel.size() == center.size() &&
        std::all_of(reply.joint_pos.begin(), reply.joint_pos.end(), [](double v) { return std::isfinite(v); }) &&
        std::all_of(reply.joint_vel.begin(), reply.joint_vel.end(), [](double v) { return std::isfinite(v); });
    std::vector<double> error;
    double max_error = 0;
    if (valid) for (size_t i=0; i<center.size(); ++i) {
      error.push_back(target[i]-reply.joint_pos[i]);
      max_error = std::max(max_error, std::abs(error.back()));
    }
    logger_->diagnostic({{"type", "pv_tracking"}, {"phase", holding ? "hold" : "trajectory"},
        {"target_period_ms", 10}, {"elapsed_ms", sent_ms}, {"rpc_ms", rpc_ms},
        {"send_gap_ms", previous_send_ms < 0 ? -1 : sent_ms-previous_send_ms},
        {"target_q", numbers(target)}, {"target_dq", numbers(velocity)},
        {"reply_q", numbers(reply.joint_pos)}, {"reply_dq", numbers(reply.joint_vel)},
        {"error_rad", numbers(error)}, {"max_error_rad", valid ? QJsonValue(max_error) : QJsonValue()},
        {"result", result}, {"valid", valid}});
    previous_send_ms = sent_ms;
    return valid && logger_->isReady() && sdk_error_count_.load() == pv_error_baseline;
  };
  logger_->event("mit_pv_trajectory_start", QString("period_ms=10;duration_s=%1;max_velocity_rad_s=0.1;path=cubic;clock=monotonic")
      .arg(pv_duration_s));
  for (;;) {
    if (friction_batch_.active() && (batch_cancel_requested_.load() || sdk_error_count_.load()!=batch_error_baseline_)) {
      leaveLowMode("batch_return_cancelled"); *reason="批次回中心已取消或发生错误。"; return false;
    }
    const double elapsed_s = pv_clock.nsecsElapsed()/1e9;
    std::vector<double> target, velocity;
    trajectory.sample(elapsed_s, target, velocity);
    carm::RobotLowData reply;
    if (!send_pv(target, velocity, false, reply)) { leaveLowMode("mit_pv_command_failed"); *reason = "Low PV 命令、反馈或日志异常。"; return false; }
    ++steps;
    if (elapsed_s >= pv_duration_s) break;
    wait_next();
  }
  double center_gap = std::numeric_limits<double>::infinity();
  const double hold_start_s = pv_clock.nsecsElapsed()/1e9;
  bool settled = false;
  while (pv_clock.nsecsElapsed()/1e9 - hold_start_s < 1.0) {
    wait_next();
    if (friction_batch_.active() && (batch_cancel_requested_.load() || sdk_error_count_.load()!=batch_error_baseline_)) {
      leaveLowMode("batch_hold_cancelled"); *reason="批次保持已取消或发生错误。"; return false;
    }
    carm::RobotLowData reply;
    if (!send_pv(center, std::vector<double>(center.size(), 0.0), true, reply)) {
      leaveLowMode("mit_pv_center_hold_failed"); *reason = "中心保持 PV 命令或状态读取失败。"; return false;
    }
    center_gap = 0.0;
    double max_velocity = 0.0;
    for (size_t i = 0; i < center.size(); ++i) {
      center_gap = std::max(center_gap, std::abs(reply.joint_pos[i] - center[i]));
      max_velocity = std::max(max_velocity, std::abs(reply.joint_vel[i]));
    }
    // Accept a single valid arrival sample; no consecutive-stability gate.
    if (center_gap <= kArrivalRad && max_velocity <= kSettledVelocityRadS) {
      settled = true;
      break;
    }
  }
  if (!settled) { leaveLowMode("mit_pv_center_not_settled"); *reason = QString("中心保持未收敛（最大差 %1 rad）。").arg(center_gap); return false; }
  std::array<double, 7> pose{}; int tool = -1;
  if (traceSdk("mit_center_forward_kinematics", [&] { return robot_->low_get_forward_kine(center, pose, tool); }) != 1 || !HandleController::validPose(pose)) {
    leaveLowMode("mit_center_fk_failed"); *reason = "中心点 FK 读取失败。"; return false;
  }
  handle_controller_.captureCenter(pose, 0.0);
  snapshot_.handle_center_captured = true;
  if (logger_) logger_->event("mit_center_reached", QString("center=%1;max_gap_rad=%2;pv_steps=%3;smooth_duration_s=%4")
      .arg(joinNumericValues(center)).arg(center_gap, 0, 'f', 6).arg(steps).arg(pv_duration_s, 0, 'f', 3));
  return true;
}

void ArmWorker::setDeadmanHeld(bool held) {
  deadman_held_ = held;
  if (!held && mit_active_) {
    stopMitHandle("physical deadman released");
    fail("实体使能开关已释放，MIT 已停止。");
  }
}

void ArmWorker::applyTuning(QJsonObject tuning, bool save) {
  if (mit_active_ || low_mode_active_ || exit_pending_ || friction_batch_.active()) {
    emit message("请先停止手柄并完成退出，再应用参数。"); return;
  }
  for (const auto* key : {"cartesian_stiffness", "cartesian_damping", "normal_joint_kp", "normal_joint_kd"}) {
    if (!tuning.value(key).isArray() || tuning.value(key).toArray().size() != 6) {
      emit message(QString("参数未应用：%1 需要六个数值。").arg(key)); return;
    }
  }
  auto merge = [&](QJsonObject base) {
    for (const auto* section : {"handle", "mit"}) {
      auto part = base.value(section).toObject();
      const QStringList keys = QString(section) == "handle" ?
          QStringList{"cartesian_stiffness", "cartesian_damping"} :
          QStringList{"normal_joint_kp", "normal_joint_kd"};
      for (const auto& key : keys) part[key] = tuning.value(key);
      if (QString(section)=="handle" && tuning.contains("near_assist"))
        part["near_assist"]=tuning.value("near_assist");
      base[section] = part;
    }
    return base;
  };
  const auto next = merge(config_);
  const auto error = validateConfig(next);
  if (!error.isEmpty()) { emit message("参数未应用：" + error); return; }
  if (save) {
    const auto path = config_.value("configuration_file").toString();
    QFile input(path);
    if (!input.open(QIODevice::ReadOnly)) { emit message("无法读取配置文件，参数未应用。"); return; }
    QJsonParseError parse;
    const auto doc = QJsonDocument::fromJson(input.readAll(), &parse);
    input.close();
    if (parse.error != QJsonParseError::NoError || !doc.isObject()) {
      emit message("配置文件无效，参数未应用。"); return;
    }
    const auto disk = merge(doc.object());
    const auto disk_error = validateConfig(disk);
    if (!disk_error.isEmpty()) { emit message("保存失败：" + disk_error); return; }
    QSaveFile output(path);
    const auto bytes = QJsonDocument(disk).toJson();
    if (!output.open(QIODevice::WriteOnly) || output.write(bytes) != bytes.size() || !output.commit()) {
      emit message("保存失败，参数未应用。"); return;
    }
  }
  auto parameters = handle_controller_.parameters();
  parameters.independent_gains = true;
  std::vector<double> kp, kd;
  for (int i = 0; i < 6; ++i) {
    parameters.cartesian_stiffness[i] = tuning.value("cartesian_stiffness").toArray()[i].toDouble();
    parameters.cartesian_damping[i] = tuning.value("cartesian_damping").toArray()[i].toDouble();
    kp.push_back(tuning.value("normal_joint_kp").toArray()[i].toDouble());
    kd.push_back(tuning.value("normal_joint_kd").toArray()[i].toDouble());
  }
  config_ = next;
  loadNearAssist(next.value("handle").toObject(), parameters);
  handle_controller_.setParameters(parameters);
  mit_controller_.setJointGains(kp, kd);
  if (logger_) logger_->diagnostic({{"type", "tuning_applied"}, {"parameters", tuning}, {"saved", save}});
  emit message(save ? "六维笛卡尔与 MIT 参数已应用并保存；下次进入手柄使用新参数。" :
                      "六维笛卡尔与 MIT 参数已应用，仅本次会话有效。");
}

void ArmWorker::applyHandleParameters(double position_scale, double rotation_scale,
                                      double position_stiffness, double position_damping,
                                      double rotation_stiffness, double rotation_damping) {
  if (mit_active_ || low_mode_active_ || exit_pending_) {
    emit message("请先停止手柄并完成退出，再应用参数，避免运行中力矩突变。"); return;
  }
  if (!std::isfinite(position_scale) || !std::isfinite(rotation_scale) ||
      !std::isfinite(position_stiffness) || !std::isfinite(position_damping) ||
      !std::isfinite(rotation_stiffness) || !std::isfinite(rotation_damping) ||
      position_scale <= 0.0 || rotation_scale <= 0.0 || position_stiffness < 0.0 ||
      position_damping < 0.0 || rotation_stiffness < 0.0 || rotation_damping < 0.0) {
    emit message("输出倍率必须为正数，刚度与阻尼必须为非负有限数。");
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
  emit message("手柄参数已应用。");
}

void ArmWorker::startGravityTest() {
  if (friction_batch_.active() && !batch_internal_call_) return;
  const auto mit = config_.value("mit").toObject();
  if (!mit.contains("normal_joint_kp") || !mit.contains("normal_joint_kd")) {
    emit message("纯重力测试需要明确配置 normal_joint_kp/kd，避免零增益污染下一次启动。");
    return;
  }
  if (!mit_active_ || mit_startup_hold_.active() || exit_pending_ || gravity_test_) {
    emit message("纯重力测试仅能在手柄启动保持结束后切入；重复请求不会重新开始。");
    return;
  }
  gravity_test_ = true;
  snapshot_.gravity_test = true;
  gravity_test_clock_.start();
  if (logger_) logger_->event("gravity_test_start", "1s smooth removal of ALL kp/kd, Cartesian and joint assistance; no automatic return");
  emit message("纯重力试验：1 秒内撤除所有刚度/阻尼和回正项。可能漂移或下坠；停止按钮沿用返回高层流程，不会自动回中。");
  publish();
}

void ArmWorker::exitHandleMode() {
  if (mit_active_) stopMitHandle("operator_return_to_high");
  else if (low_mode_active_ && !exit_pending_) leaveLowMode("operator_low_exit");
  publish();
}

void ArmWorker::startFrictionProbe(int joint, int direction) {
  if (friction_batch_.active() && !batch_internal_call_) return;
  if (!mit_active_ || exit_pending_ || mit_startup_hold_.active() || friction_probe_.active ||
      !gravity_test_ || gravity_test_clock_.elapsed()<1500 || !snapshot_.gravity_only_unlimited) {
    emit message("起动力矩测试需先进入纯重力并稳定，不能并发测试。"); return;
  }
  carm::RobotLowData low;
  if (refreshLow(low)!=1 || low.joint_pos.size()!=6 ||
      !low.status.arm_connected || !low.status.arm_enable || !low.status.arm_status || low.status.error_code) {
    emit message("测试拒绝：状态无效。"); return;
  }
  for (double q:low.joint_pos) if (!std::isfinite(q)) {
    emit message("测试拒绝：关节位置无效。"); return;
  }
  if (friction_batch_.active() && !friction_batch_.withinOrigin(low.joint_pos)) {
    emit message("测试拒绝：已偏离本轮测试起点超过 0.05 rad。"); return;
  }
  if (!friction_probe_.start(joint,direction,low.joint_pos,mit_clock_.nsecsElapsed()/1e9)) return;
  logger_->diagnostic({{"type","friction_probe_start"},{"joint",joint+1},{"direction",direction},
      {"q_start",numbers(low.joint_pos)},{"limit_nm",FrictionProbe::axisLimit(joint)},
      {"ramp_nm_s",FrictionProbe::rampRate(joint)},{"profile","wrist_accumulated_v3"},
      {"note","single supervised direction; outcome is a candidate, not calibrated friction"}});
  emit message(QString("J%1 %2 起动力矩测试：先观察 1 秒，再以 %3 N·m/s 加力，上限 %4 N·m。")
      .arg(joint+1).arg(direction>0?"正向":"反向").arg(FrictionProbe::rampRate(joint)).arg(FrictionProbe::axisLimit(joint)));
}

void ArmWorker::abortFrictionBatch(const QString& reason) {
  if (!friction_batch_.active()) return;
  batch_timer_.stop();
  friction_batch_.cancel();
  if (logger_) logger_->diagnostic({{"type","friction_batch_aborted"},{"reason",reason},
      {"completed",friction_batch_.index()},{"thresholds_nm",numbers(friction_batch_.thresholds)},
      {"identified",numbers(friction_batch_.identified)}});
  emit message("整批辨识已中止："+reason+"；不重试、不自动应用补偿。");
}

void ArmWorker::startFrictionBatch() {
  if (friction_batch_.active() || friction_probe_.active || !mit_active_ || exit_pending_ ||
      !gravity_test_ || !snapshot_.gravity_only_unlimited || gravity_test_clock_.elapsed()<1500) {
    emit message("批次需从稳定的纯重力模式启动，以当前姿态作为本轮起点。"); return;
  }
  carm::RobotLowData low;
  if (refreshLow(low)!=1 || low.joint_pos.size()!=6 || !low.status.arm_connected ||
      !low.status.arm_enable || !low.status.arm_status || low.status.error_code) {
    emit message("批次拒绝：无法读取有效的测试起始状态。"); return;
  }
  if (!friction_batch_.start(low.joint_pos)) {
    emit message("批次拒绝：测试起始关节数据无效。"); return;
  }
  batch_cancel_requested_.store(false);
  batch_error_baseline_=sdk_error_count_.load();
  batch_model_hash_=use_urdf_gravity_?urdf_gravity_.sha256():QString();
  batch_previous_q_.clear(); batch_settle_q_.clear();
  batch_phase_clock_.start();
  logger_->diagnostic({{"type","friction_batch_start"},{"total",36},{"model_sha256",batch_model_hash_},
      {"q_start",numbers(friction_batch_.origin())},{"gravity_source",gravity_compensation_active_?"urdf_compensated":(use_urdf_gravity_?"urdf":"sdk")},
      {"order","J1 (+,-) x3; ...; J6 (+,-) x3"}});
  batch_internal_call_=true;
  startFrictionProbe(0,1);
  batch_internal_call_=false;
  if (!friction_probe_.active) { abortFrictionBatch("first_probe_refused"); return; }
  batch_timer_.start(); publish();
}

void ArmWorker::advanceFrictionBatch() {
  if (!friction_batch_.active()) { batch_timer_.stop(); return; }
  if (batch_cancel_requested_.load() || !robot_ || !robot_->is_connected() || !mit_active_ ||
      !gravity_test_ || exit_pending_ || sdk_error_count_.load()!=batch_error_baseline_ ||
      snapshot_.safety!=SafetyState::Normal) {
    stopMitHandle("batch_cancel_or_invalid_session"); publish(); return;
  }
  const int timeout = friction_batch_.phase==FrictionBatch::Phase::Probing ? 30000 : 5000;
  if (batch_phase_clock_.elapsed()>timeout) {
    stopMitHandle("batch_motion_or_settling_timeout"); publish();
  }
}

void ArmWorker::emergencyStop() {
  abortFrictionBatch("emergency_stop");
  friction_probe_.cancel();
  mit_startup_hold_.reset();
  // Do not delay the emergency request behind ordinary stop/mode-change RPCs.
  mit_timer_.stop();
  mit_active_ = false;
  if (robot_) robot_->emergency_stop();
  modes_.exitHandle();
  snapshot_.application_mode = modes_.mode();
  snapshot_.safety = SafetyState::EmergencyStop;
  emit message("已请求软件急停。如存在即时危险，请使用实体急停。");
  if (logger_) logger_->command("emergency_stop");
  publish();
}

void ArmWorker::pollRobot() {
  QElapsedTimer timer;
  timer.start();
  if (!robot_) return;
  if (minimal_exit_observing_) {
    const auto status = robot_->get_status();
    const bool connected = robot_->is_connected();
    const bool no_errors = sdk_error_count_.load() == minimal_exit_error_baseline_;
    // Allow status propagation initially; latch subsequent anomalies.
    if (minimal_exit_clock_.elapsed() >= 300)
      minimal_exit_observation_ok_ = minimal_exit_observation_ok_ && connected &&
          status.state >= 0 && (status.fsm_state == 0 || status.fsm_state == 1) && no_errors;
    logger_->diagnostic({{"type", "minimal_exit_observation"},
        {"elapsed_ms", minimal_exit_clock_.elapsed()}, {"connected", connected},
        {"enabled", status.servo_status}, {"state", status.state}, {"fsm", status.fsm_state},
        {"new_sdk_errors", !no_errors}});
    snapshot_.servo = status.servo_status ? ServoState::Enabled : ServoState::Disabled;
    snapshot_.controller_state = status.state;
    snapshot_.vendor_fsm_state = status.fsm_state;
    snapshot_.connection = connected ? ConnectionState::Connected : ConnectionState::Disconnected;
    if (minimal_exit_clock_.elapsed() >= 5000) {
      minimal_exit_observing_ = false;
      const bool passed = minimal_exit_commands_ok_ && minimal_exit_observation_ok_ && no_errors;
      exit_pending_ = !passed;
      if (!passed) snapshot_.safety = SafetyState::Fault;
      logger_->event("minimal_exit_finished", passed ?
          QString("high_transport_observed;enabled=%1;high motion NOT tested").arg(status.servo_status) :
          "failed;operation gate remains locked;no automatic recovery");
      emit message(passed ? (status.servo_status ?
          "已观察到高层模式且无新增错误；注意：高层仍报告已使能，不代表安全下使能！" :
          "已观察到高层模式且报告未使能；尚未验证高层运动。") :
          "最小退出观察失败，保持锁定；没有追加恢复或运动操作。");
    }
    publish(); return;
  }
  snapshot_.connection = robot_->is_connected() ? ConnectionState::Connected : ConnectionState::Disconnected;
  if (snapshot_.connection != ConnectionState::Connected) return fail("控制器连接已断开。");
  // Low-mode snapshots are produced by the control loop. Never advance its
  // filter/damping state from the independent GUI polling timer.
  if (mit_active_) {
    publish();
    return;
  }
  const auto status = robot_->get_status();
  snapshot_.servo = status.servo_status ? ServoState::Enabled : ServoState::Disabled;
  snapshot_.controller_state = status.state;
  snapshot_.vendor_fsm_state = status.fsm_state;
  snapshot_.vendor_debug_mode = status.on_debug_mode;
  snapshot_.application_mode = modes_.mode();
  snapshot_.joint_position = robot_->get_joint_pos();
  snapshot_.joint_velocity = robot_->get_joint_vel();
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
  if (snapshot_.safety == SafetyState::Normal &&
      !safety_.isSafeToContinue(snapshot_, &reason)) return fail(reason);
  if (logger_) logger_->performance("state_poll", timer.nsecsElapsed() / 1'000'000.0);
  publish();
}

void ArmWorker::runMitCycle() {
  if (friction_batch_.active() && (batch_cancel_requested_.load() || sdk_error_count_.load()!=batch_error_baseline_)) {
    stopMitHandle("batch_cancel_or_sdk_error"); return;
  }
  if (!mit_active_ || !robot_) return;
  if (!logger_ || !logger_->isReady()) return fail("MIT 已停止：诊断日志写入失败。");
  QElapsedTimer timer;
  timer.start();
  if (!mit_last_success_.isValid() || mit_last_success_.elapsed() > mit_watchdog_ms_) {
    stopMitHandle(QString("MIT watchdog expired after %1 ms without a successful cycle.")
                      .arg(mit_last_success_.isValid() ? mit_last_success_.elapsed() : -1));
    return;
  }
  carm::RobotLowData low;
  if (refreshLow(low) != 1 || low.joint_pos.empty()) {
    registerMitFailure("low_refresh failed");
    return;
  }
  if (!low.status.arm_connected || !low.status.arm_enable || !low.status.arm_status ||
      low.status.error_code != 0 || (low.status.arm_mode >= 0 && low.status.arm_mode != 2)) {
    if (logger_) logger_->event("mit_invalid_low_status",
        QString("connected=%1;enabled=%2;healthy=%3;mode=%4;error_code=%5;error_msg=%6")
            .arg(low.status.arm_connected).arg(low.status.arm_enable).arg(low.status.arm_status)
            .arg(low.status.arm_mode).arg(low.status.error_code).arg(QString::fromStdString(low.status.error_msg)));
    return fail("MIT 低层状态异常：连接、使能、健康状态或模式检查失败。");
  }
  if (friction_batch_.active()) {
    if (low.joint_pos.size()!=6 || friction_batch_.origin().size()!=6) {
      stopMitHandle("batch_invalid_state"); return;
    }
    const double now=mit_clock_.nsecsElapsed()/1e9;
    const double dt=now-batch_previous_time_;
    if (!friction_batch_.withinOrigin(low.joint_pos)) {
      stopMitHandle("batch_start_neighborhood_exceeded"); return;
    }
    for (size_t i=0;i<6;++i) {
      if (!batch_previous_q_.empty() && (dt<=0 || dt>0.1 || std::abs(low.joint_pos[i]-batch_previous_q_[i])/dt>0.3)) {
        stopMitHandle("batch_speed_or_timing_guard"); return;
      }
    }
    batch_previous_q_=low.joint_pos; batch_previous_time_=now;
  }
  std::array<double, 7> pose{};
  int tool = -1;
  carm::RobotMatrix jacobian;
  std::vector<double> mass, coriolis, gravity(low.joint_pos.size(), 0.0), zero_acc(low.joint_pos.size(), 0.0);
  if (traceSdk("low_get_forward_kine", [&] { return robot_->low_get_forward_kine(low.joint_pos, pose, tool); }) != 1 ||
      traceSdk("low_get_jacobian", [&] { return robot_->low_get_jacobian(low.joint_pos, tool, jacobian); }) != 1 ||
      traceSdk("low_get_dynamics", [&] { return robot_->low_get_dynamics(low.joint_pos, low.joint_vel, zero_acc, tool, mass, coriolis, gravity); }) != 1) {
    registerMitFailure("MIT kinematic/dynamic query failed");
    return;
  }
  QString safety_reason;
  const auto sdk_gravity=gravity;
  if (use_urdf_gravity_) {
    gravity=urdf_gravity_.compute(low.joint_pos);
    if (gravity.size()!=low.joint_pos.size()) return fail("URDF 重力计算失败，已停止，不自动回退 SDK。");
  }
  const auto baseline_gravity=gravity;
  std::vector<double> compensated_physical, gravity_delta, compensated_total;
  if (gravity_compensation_active_) {
    compensated_physical=compensated_gravity_.compute(low.joint_pos);
    if (compensated_physical.size()!=gravity.size()) return fail("Compensated gravity calculation invalid");
    compensated_total=compensated_physical;
    gravity_delta.resize(gravity.size());
    for (size_t i=0;i<gravity.size();++i) {
      compensated_total[i]+=gravity_residual_[i];
      gravity_delta[i]=compensated_total[i]-gravity[i];
    }
    // The validated model is the commanded gravity source.  It is intentionally
    // not time-, displacement-, or velocity-gated like the former trial path.
    gravity=compensated_total;
  }
  if (!safety_.isSafeForMit(low.joint_pos, low.joint_vel, &safety_reason)) {
    stopMitHandle(QString("MIT safety stop: %1").arg(safety_reason));
    fail(safety_reason);
    return;
  }
  const auto handle = handle_controller_.update(pose, mit_clock_.nsecsElapsed() / 1e9);
  const bool startup_was_active = mit_startup_hold_.active();
  const double startup_weight = mit_startup_hold_.update(
      mit_clock_.nsecsElapsed() / 1e9, low.joint_pos, low.joint_vel);
  if (startup_weight < 0.0) {
    stopMitHandle("MIT startup hold invalid-state/drift abort");
    emit message("MIT 启动状态无效或超出保持位移范围，已停止；请检查日志。");
    return;
  }
  if (startup_was_active && !mit_startup_hold_.active()) {
    logger_->event("mit_startup_hold_finished", "normal compliant handle active");
    emit message("启动保持与平滑释放完成，已进入柔顺手柄模式。");
  }
  const double progress = gravity_test_ ? std::clamp(gravity_test_clock_.nsecsElapsed() / 1e9, 0.0, 1.0) : 0.0;
  const double interaction_scale = 1.0 - progress * progress * (3.0 - 2.0 * progress);
  std::vector<double> probe_torque;
  if (friction_probe_.active) {
    friction_probe_.update(low.joint_pos,mit_clock_.nsecsElapsed()/1e9);
    if (!friction_probe_.active) {
      logger_->diagnostic({{"type","friction_probe_result"},{"joint",friction_probe_.joint+1},
          {"direction",friction_probe_.direction},{"outcome",QString::fromStdString(friction_probe_.outcome)},
          {"candidate_threshold_nm",friction_probe_.threshold},{"last_applied_nm",friction_probe_.applied},
          {"q_end",numbers(low.joint_pos)}});
      emit message("起动力矩测试结束："+QString::fromStdString(friction_probe_.outcome)+"；仅记录候选结果，未启用补偿。");
      if (friction_batch_.active()) {
        const bool identified=friction_probe_.outcome=="onset_candidate";
        const bool unmeasured=friction_probe_.outcome=="no_onset_below_limit";
        if ((identified && friction_batch_.accept(friction_probe_.threshold,friction_probe_.applied)) ||
            (unmeasured && friction_batch_.unidentified(friction_probe_.applied))) {
          logger_->diagnostic({{"type","friction_batch_sample"},{"completed",friction_batch_.index()},
              {"joint",friction_probe_.joint+1},{"direction",friction_probe_.direction},
              {"identified",identified},{"outcome",QString::fromStdString(friction_probe_.outcome)},
              {"threshold_nm",identified ? QJsonValue(friction_probe_.threshold) : QJsonValue(QJsonValue::Null)}});
          batch_phase_clock_.restart();
          batch_settle_q_=low.joint_pos;
          batch_stable_since_=mit_clock_.nsecsElapsed()/1e9;
        } else {
          stopMitHandle("invalid_probe_outcome: "+QString::fromStdString(friction_probe_.outcome)); return;
        }
      } else {
        stopMitHandle("friction_probe_finished"); return;
      }
    }
    probe_torque.assign(6,0.0);
    probe_torque[friction_probe_.joint]=friction_probe_.requested+friction_probe_.braking;
  }
  if (friction_batch_.active() && friction_batch_.phase!=FrictionBatch::Phase::Probing) {
    probe_torque.assign(6,0.0);
    const double now=mit_clock_.nsecsElapsed()/1e9;
    if (friction_batch_.phase==FrictionBatch::Phase::Releasing) {
      probe_torque[(friction_batch_.index()-1)/6]=friction_batch_.withdraw(handle.dt);
      batch_settle_q_=low.joint_pos; batch_stable_since_=now;
    } else {
      bool stable=true;
      for (size_t i=0;i<6;++i) stable=stable && std::abs(low.joint_pos[i]-batch_settle_q_[i])<=0.0008;
      if (!stable) { batch_settle_q_=low.joint_pos; batch_stable_since_=now; }
      if (now-batch_stable_since_>=0.5 && snapshot_.gravity_only_unlimited) {
        friction_batch_.settled(); batch_phase_clock_.restart();
        if (!friction_batch_.active()) {
          batch_timer_.stop();
          logger_->diagnostic({{"type","friction_batch_complete"},{"thresholds_nm",numbers(friction_batch_.thresholds)},
              {"identified",numbers(friction_batch_.identified)},
              {"model_sha256",batch_model_hash_},{"note","thresholds with identified=0 are placeholders, NOT measurements; gravity remains active; compensation NOT applied"}});
          emit message("36 次测试完成（可能含未辨识项）；附加测试力矩已撤除，保持纯重力模式，不自动回中心。");
        } else {
          batch_internal_call_=true;
          startFrictionProbe(friction_batch_.joint(),friction_batch_.direction());
          batch_internal_call_=false;
          if (!friction_probe_.active) { stopMitHandle("batch_next_probe_refused"); return; }
        }
      }
    }
  }
  auto command = mit_controller_.compute(handle, low.joint_pos, low.joint_vel, gravity,
                                         jacobian.data, jacobian.rows, jacobian.cols, interaction_scale, probe_torque,
                                         probe_torque.empty()?0.0:FrictionProbe::limit_nm);
  if (friction_probe_.active || friction_batch_.active()) {
    bool clipped=!command.valid;
    for (size_t i=0;command.valid && i<6;++i)
      clipped=clipped || command.torque_saturated[i] || std::abs(command.torque[i]-gravity[i]-probe_torque[i])>0.03;
    if (clipped) {
      logger_->event("friction_probe_invalid","total/rate/interaction clipping or invalid command; no calibration result");
      stopMitHandle("friction_probe_clipped"); return;
    }
  }
  auto applied_wrench = handle.virtual_wrench;
  for (auto& value : applied_wrench) value *= interaction_scale;
  bool gravity_only_settled = gravity_test_ && interaction_scale == 0.0 && command.valid;
  for (size_t i = 0; gravity_only_settled && i < command.torque.size(); ++i)
    gravity_only_settled = std::abs(command.torque[i] - gravity[i]) < 1e-6;
  if (command.valid) mit_startup_hold_.apply(startup_weight, command.position);
  if (logger_) logger_->diagnostic({{"type", "control_math"}, {"next_frame", diagnostic_frame_ + 1},
      {"gravity", numbers(gravity)}, {"jacobian", numbers(jacobian.data)},
      {"gravity_source", gravity_compensation_active_ ? "urdf_compensated" : (use_urdf_gravity_ ? "urdf" : "sdk")}, {"sdk_gravity", numbers(sdk_gravity)},
      {"baseline_gravity",numbers(baseline_gravity)},
      {"gravity_compensation_active",gravity_compensation_active_},
      {"compensated_physical_gravity",numbers(compensated_physical)},
      {"gravity_compensation_delta",numbers(gravity_delta)},{"compensated_gravity_total",numbers(compensated_total)},
      {"rows", jacobian.rows}, {"cols", jacobian.cols}, {"tool", tool},
      {"pose", numbers(pose)}, {"wrench", numbers(applied_wrench)},
      {"handle_error", numbers(handle.raw_error)}, {"handle_velocity", numbers(handle.velocity)},
      {"return_assist_requested", numbers(handle.return_assist)},
      {"assist_region", numbers(handle.assist_region)},
      {"gravity_test", gravity_test_}, {"interaction_scale", interaction_scale},
      {"gravity_only_unlimited", gravity_only_settled},
      {"gravity_torque", numbers(command.gravity_torque)}, {"cartesian_torque", numbers(command.cartesian_torque)},
      {"joint_assist_torque", numbers(command.joint_assist_torque)},
      {"static_friction_torque", numbers(command.static_friction_torque)},
      {"interaction_torque", numbers(command.interaction_torque)},
      {"friction_probe_active",friction_probe_.active},{"diagnostic_torque",numbers(probe_torque)},
      {"probe_excitation_nm",friction_probe_.active?friction_probe_.requested:0.0},
      {"probe_braking_nm",friction_probe_.active?friction_probe_.braking:0.0},
      // This is the post-clamp, post-rate-limit vector passed unchanged to
      // low_mit_command; distinguish it from the three requested components.
      {"sent_torque", numbers(command.torque)},
      {"startup_hold_weight", startup_weight},
      {"torque_saturated", numbers(command.torque_saturated)}, {"valid", command.valid}});
  // Keep q and FK from the same measurement; low_mit_command overwrites low.
  snapshot_.joint_position = low.joint_pos;
  snapshot_.joint_velocity = low.joint_vel;
  snapshot_.cartesian_pose = pose;
  int command_result = -1;
  if (command.valid) {
    command_result = sendMit(command, low);
  }
  if (logger_ && (mit_command_attempts_ < 3 || command_result != 1)) {
    const auto torque_range = command.torque.empty()
                                  ? std::pair<double, double>{0.0, 0.0}
                                  : std::pair<double, double>{*std::min_element(command.torque.begin(), command.torque.end()),
                                                              *std::max_element(command.torque.begin(), command.torque.end())};
    logger_->event("mit_command_attempt",
                   QString("attempt=%1;valid=%2;result=%3;dof=%4;dt=%5;tau_min=%6;tau_max=%7;kp=%8;kd=%9")
                       .arg(mit_command_attempts_ + 1).arg(command.valid).arg(command_result)
                       .arg(command.position.size()).arg(handle.dt, 0, 'f', 6)
                       .arg(torque_range.first, 0, 'f', 6).arg(torque_range.second, 0, 'f', 6)
                       .arg(command.kp.empty() ? 0.0 : command.kp.front(), 0, 'f', 6)
                       .arg(command.kd.empty() ? 0.0 : command.kd.front(), 0, 'f', 6));
  }
  ++mit_command_attempts_;
  if (!command.valid || command_result != 1) {
    if (logger_) {
      logger_->event("mit_rejected_reply", QString("valid=%1;result=%2;error_code=%3;error_msg=%4;mode=%5;enabled=%6;healthy=%7")
          .arg(command.valid).arg(command_result).arg(low.status.error_code)
          .arg(QString::fromStdString(low.status.error_msg)).arg(low.status.arm_mode)
          .arg(low.status.arm_enable).arg(low.status.arm_status));
      logger_->performance("mit_failed_cycle", timer.nsecsElapsed() / 1'000'000.0);
    }
    registerMitFailure("MIT command rejected");
    return;
  }
  mit_last_accepted_position_ = command.position;
  if (friction_probe_.active)
    friction_probe_.applied=command.torque[friction_probe_.joint]-gravity[friction_probe_.joint];
  if (timer.elapsed() > mit_watchdog_ms_) {
    stopMitHandle(QString("MIT cycle exceeded watchdog budget: %1 ms.").arg(timer.elapsed()));
    return;
  }
  mit_failures_ = 0;
  mit_last_success_.restart();
  snapshot_.gravity_only_unlimited = gravity_only_settled;
  snapshot_.handle_command = handle.command;
  snapshot_.virtual_wrench = applied_wrench;
  if (logger_) {
    auto logged_handle = handle;
    logged_handle.virtual_wrench = applied_wrench;
    logger_->handle(logged_handle);
    logger_->performance("mit_cycle", timer.nsecsElapsed() / 1'000'000.0);
  }
}

void ArmWorker::registerMitFailure(const QString& reason) {
  if (friction_probe_.active || friction_batch_.active()) { stopMitHandle("friction_probe_rpc_failure: "+reason); return; }
  ++mit_failures_;
  if (logger_) logger_->event("mit_cycle_failure", QString("%1 (%2/%3)")
                              .arg(reason).arg(mit_failures_).arg(mit_max_failures_));
  if (mit_failures_ >= mit_max_failures_) {
    stopMitHandle(QString("MIT stopped after %1 consecutive failures: %2").arg(mit_failures_).arg(reason));
  }
}

bool ArmWorker::leaveLowMode(const QString& context) {
  if (!robot_ || !low_mode_active_) return true;
  if (exit_pending_) return false;
  exit_pending_ = true;
  handle_controller_.clearCenter();
  snapshot_.handle_center_captured = false;
  snapshot_.handle_command.fill(0.0);
  snapshot_.virtual_wrench.fill(0.0);
  minimal_exit_error_baseline_ = sdk_error_count_.load();
  const int disabled = traceSdk("minimal_exit_low_disable", [&] { return robot_->low_set_servo_enable(false); });
  const int transport = disabled == 1 ?
      traceSdk("minimal_exit_low_off", [&] { return robot_->set_low_mode(false); }) : -1;
  if (disabled == 1) low_servo_active_ = false;
  if (transport == 1) low_mode_active_ = false;
  minimal_exit_commands_ok_ = disabled == 1 && transport == 1;
  minimal_exit_observation_ok_ = true;
  minimal_exit_observing_ = true;
  minimal_exit_clock_.start();
  if (logger_) logger_->event("minimal_exit_start",
      QString("context=%1;disable=%2;low_off=%3;read_only_observation_ms=5000")
          .arg(context).arg(disabled).arg(transport));
  emit message("已请求低层下使能并关闭透传，只读观察 5 秒；高层使能状态以反馈为准。");
  return minimal_exit_commands_ok_;
}

bool ArmWorker::enterMitLowMode(QString* reason) {
  // The SDK's low-level example requires this order: enter low transport, enable its servo,
  // then select its robot mode before issuing low_mit_command().
  const int low_mode_result = low_mode_active_ ? 1 : traceSdk("set_low_mode", [&] { return robot_->set_low_mode(true); });
  if (logger_) logger_->event("low_mode_entry", QString("set_low_mode_true=%1").arg(low_mode_result));
  if (low_mode_result != 1) {
    *reason = QString("Controller rejected low-mode entry (%1).").arg(low_mode_result);
    return false;
  }
  low_mode_active_ = true;
  // set_low_mode() acknowledges before the controller has necessarily left highMode. Automated
  // tests reproduced vendor error 3001 when setEnable followed after ~1 ms; 10 ms was repeatable.
  if (low_mode_result == 1 && !low_servo_active_) QThread::msleep(10);
  // Complete expensive, non-commanding preflight BEFORE selecting low MIT mode.
  const int servo_result = low_servo_active_ ? 1 : traceSdk("low_set_servo_enable", [&] { return robot_->low_set_servo_enable(true); });
  if (logger_) logger_->event("low_mode_entry", QString("low_set_servo_enable_true=%1").arg(servo_result));
  if (servo_result != 1) {
    leaveLowMode("mit_entry_failed");
    *reason = QString("Controller rejected low-level servo enable (%1).").arg(servo_result);
    return false;
  }
  low_servo_active_ = true;
  // Run the multi-RPC FK/Jacobian check while PV still supports the arm,
  // not in the interval between selecting MIT and sending its first command.
  if (!verifyMitJacobian(reason)) {
    leaveLowMode("mit_jacobian_verification_failed");
    return false;
  }
  const int robot_mode_result = traceSdk("low_set_robot_mode", [&] { return robot_->low_set_robot_mode(2); });
  mode_ack_clock_.start();
  send_clock_.invalidate();
  diagnostic_frame_ = 0;
  snapshot_.first_send_delay_ms = -1;
  snapshot_.last_send_gap_ms = -1;
  snapshot_.mit_frames = 0;
  if (logger_) logger_->event("low_mode_entry", QString("low_set_robot_mode_mit_2=%1").arg(robot_mode_result));
  if (robot_mode_result != 1) {
    leaveLowMode("mit_entry_failed");
    *reason = QString("Controller rejected low-level robot mode (%1).").arg(robot_mode_result);
    return false;
  }
  carm::RobotLowData low;
  const int refresh_result = refreshLow(low);
  if (logger_) {
    logger_->event("low_mode_entry",
                   QString("low_refresh=%1;arm_connected=%2;arm_enable=%3;arm_status=%4;arm_mode=%5;dof=%6")
                       .arg(refresh_result).arg(low.status.arm_connected).arg(low.status.arm_enable)
                       .arg(low.status.arm_status).arg(low.status.arm_mode).arg(low.joint_pos.size()));
  }
  // The SDK's mode-1 example is specifically for low_pv_command(). MIT commands require
  // the low-level arm and each arm servo to be in mode 2. Some firmware omits arm_mode
  // from low_refresh and the SDK then exposes its sentinel value -1.
  const bool conflicting_reported_mode = low.status.arm_mode >= 0 && low.status.arm_mode != 2;
  if (refresh_result != 1 || !low.status.arm_connected || !low.status.arm_enable ||
      !low.status.arm_status || conflicting_reported_mode || low.joint_pos.empty()) {
    leaveLowMode("mit_entry_state_invalid");
    *reason = QString("Low-level MIT state was not confirmed: refresh=%1, connected=%2, enabled=%3, "
                      "status=%4, mode=%5, dof=%6.")
                  .arg(refresh_result).arg(low.status.arm_connected).arg(low.status.arm_enable)
                  .arg(low.status.arm_status).arg(low.status.arm_mode).arg(low.joint_pos.size());
    return false;
  }
  if (low.status.arm_mode < 0 && logger_) {
    logger_->event("low_mode_entry", "arm_mode unavailable from firmware; using acknowledged MIT-mode request");
  }
  carm::ServoStatus servo_status;
  int servo_status_result = -1;
  int servo_status_polls = 0;
  bool servo_mode_confirmed = false;
  QElapsedTimer servo_mode_timer;
  servo_mode_timer.start();
  while (servo_mode_timer.elapsed() < 50) {
    ++servo_status_polls;
    servo_status_result = readServos(servo_status);
    servo_mode_confirmed = servo_status_result == 1 &&
                           servo_status.fsmMode.size() >= low.joint_pos.size() &&
                           servo_status.isServoEnable.size() >= low.joint_pos.size() &&
                           servo_status.isConnected.size() >= low.joint_pos.size() &&
                           servo_status.motorErrorCode.size() >= low.joint_pos.size();
    for (size_t joint = 0; servo_mode_confirmed && joint < low.joint_pos.size(); ++joint) {
      servo_mode_confirmed = servo_status.fsmMode[joint] == 2 && servo_status.isServoEnable[joint] &&
                             servo_status.isConnected[joint] && servo_status.motorErrorCode[joint] == 0;
    }
    if (servo_mode_confirmed) break;
    QThread::msleep(1);
  }
  if (logger_) {
    logger_->event("mit_servo_status",
                   QString("result=%1;confirmed=%2;polls=%3;elapsed_ms=%4;dof=%5;mitKp=%6;mitKd=%7;"
                           "fsmMode=%8;isServoEnable=%9;isConnected=%10;motorErrorCode=%11")
                       .arg(servo_status_result).arg(servo_mode_confirmed).arg(servo_status_polls)
                       .arg(servo_mode_timer.elapsed()).arg(servo_status.dof)
                       .arg(joinNumericValues(servo_status.mitKp)).arg(joinNumericValues(servo_status.mitKd))
                       .arg(joinNumericValues(servo_status.fsmMode)).arg(joinNumericValues(servo_status.isServoEnable))
                       .arg(joinNumericValues(servo_status.isConnected)).arg(joinNumericValues(servo_status.motorErrorCode)));
  }
  if (servo_status_result != 1 || !servo_mode_confirmed) {
    leaveLowMode("mit_servo_status_failed");
    *reason = QString("Unable to confirm all arm servos in low-level MIT mode: status=%1, polls=%2, elapsed=%3 ms.")
                  .arg(servo_status_result).arg(servo_status_polls).arg(servo_mode_timer.elapsed());
    return false;
  }
  const auto mit = config_.value("mit").toObject();
  if (mit.contains("normal_joint_kp") && mit.contains("normal_joint_kd")) {
    std::vector<double> kp, kd;
    for (const auto value : mit.value("normal_joint_kp").toArray()) kp.push_back(value.toDouble());
    for (const auto value : mit.value("normal_joint_kd").toArray()) kd.push_back(value.toDouble());
    if (kp.size() != low.joint_pos.size() || kd.size() != low.joint_pos.size()) {
      leaveLowMode("mit_configured_gain_invalid");
      *reason = "Configured normal MIT gains do not match measured arm DOF.";
      return false;
    }
    mit_controller_.setJointGains(kp, kd);
    if (logger_) logger_->event("mit_gain_source", QString("explicit_normal_config;kp=%1;kd=%2")
        .arg(joinNumericValues(kp)).arg(joinNumericValues(kd)));
  } else if (mit.value("joint_kp").toDouble(0.0) == 0.0 && mit.value("joint_kd").toDouble(0.0) == 0.0) {
    const auto arm_dof = low.joint_pos.size();
    const bool status_sizes_valid = servo_status.mitKp.size() >= arm_dof && servo_status.mitKd.size() >= arm_dof &&
                                    servo_status.fsmMode.size() >= arm_dof &&
                                    servo_status.isServoEnable.size() >= arm_dof &&
                                    servo_status.isConnected.size() >= arm_dof &&
                                    servo_status.motorErrorCode.size() >= arm_dof;
    if (!status_sizes_valid) {
      leaveLowMode("mit_vendor_gain_invalid");
      *reason = QString("Vendor MIT gain/status vectors are shorter than the arm DOF (%1).").arg(arm_dof);
      return false;
    }
    std::vector<double> vendor_kp(servo_status.mitKp.begin(), servo_status.mitKp.begin() + arm_dof);
    std::vector<double> vendor_kd(servo_status.mitKd.begin(), servo_status.mitKd.begin() + arm_dof);
    const bool gains_valid = std::all_of(vendor_kp.begin(), vendor_kp.end(),
                                         [](double value) { return std::isfinite(value) && value >= 0.0; }) &&
                             std::all_of(vendor_kd.begin(), vendor_kd.end(),
                                         [](double value) { return std::isfinite(value) && value >= 0.0; }) &&
                             std::any_of(vendor_kp.begin(), vendor_kp.end(), [](double value) { return value > 0.0; }) &&
                             std::any_of(vendor_kd.begin(), vendor_kd.end(), [](double value) { return value > 0.0; });
    bool servos_ready = true;
    for (size_t joint = 0; joint < arm_dof; ++joint) {
      servos_ready = servos_ready && servo_status.isServoEnable[joint] && servo_status.isConnected[joint] &&
                     servo_status.motorErrorCode[joint] == 0 && servo_status.fsmMode[joint] == 2;
    }
    if (!gains_valid || !servos_ready) {
      leaveLowMode("mit_vendor_gain_invalid");
      *reason = "Vendor MIT gains are invalid, or an arm servo is not connected, enabled, error-free, and in MIT mode 2.";
      return false;
    }
    mit_controller_.setJointGains(vendor_kp, vendor_kd);
    if (logger_) {
      logger_->event("mit_gain_source", QString("vendor_servo_status;arm_dof=%1;kp=%2;kd=%3")
                                           .arg(arm_dof).arg(joinNumericValues(vendor_kp))
                                           .arg(joinNumericValues(vendor_kd)));
    }
  } else {
    mit_controller_.setJointGains({}, {});
    if (logger_) logger_->event("mit_gain_source", "scalar_config");
  }
  return true;
}

bool ArmWorker::verifyMitJacobian(QString* reason) {
  // Query FK at virtual joint vectors; these calls do not command motion.
  carm::RobotLowData low;
  carm::RobotMatrix jacobian;
  std::array<double, 7> pose{};
  int tool = -1;
  if (refreshLow(low) != 1 || low.joint_pos.empty() ||
      traceSdk("low_get_forward_kine", [&] { return robot_->low_get_forward_kine(low.joint_pos, pose, tool); }) != 1 ||
      traceSdk("low_get_jacobian", [&] { return robot_->low_get_jacobian(low.joint_pos, tool, jacobian); }) != 1) {
    *reason = "MIT Jacobian verification: failed to obtain FK/Jacobian.";
    return false;
  }
  const int dof = static_cast<int>(low.joint_pos.size());
  bool spatial = jacobian.rows == 6 && jacobian.cols == dof;
  bool transposed = jacobian.rows == dof && jacobian.cols == 6;
  if ((!spatial && !transposed) || jacobian.data.size() != static_cast<size_t>(6 * dof)) {
    *reason = "MIT Jacobian verification: invalid dimensions.";
    return false;
  }
  constexpr double step = 1e-4;
  double spatial_error = 0.0, transposed_error = 0.0;
  for (int joint = 0; joint < dof; ++joint) {
    auto plus_q = low.joint_pos, minus_q = low.joint_pos;
    plus_q[joint] += step;
    minus_q[joint] -= step;
    std::array<double, 7> plus{}, minus{};
    int plus_tool = tool, minus_tool = tool;
    if (traceSdk("low_get_forward_kine", [&] { return robot_->low_get_forward_kine(plus_q, plus, plus_tool); }) != 1 ||
        traceSdk("low_get_forward_kine", [&] { return robot_->low_get_forward_kine(minus_q, minus, minus_tool); }) != 1 ||
        plus_tool != tool || minus_tool != tool) {
      *reason = "MIT Jacobian verification: perturbed FK failed or tool changed.";
      return false;
    }
    for (const auto& p : {plus, minus}) {
      double norm = 0.0;
      for (int axis = 3; axis < 7; ++axis) norm += p[axis] * p[axis];
      if (!std::all_of(p.begin(), p.end(), [](double v) { return std::isfinite(v); }) ||
          std::abs(norm - 1.0) > 0.01) {
        *reason = "MIT Jacobian verification: invalid FK pose/quaternion.";
        return false;
      }
    }
    HandleController differential;
    differential.captureCenter(minus, 0.0);
    const auto error = differential.update(plus, 0.01).raw_error;
    for (int axis = 0; axis < 6; ++axis) {
      const double derivative = error[axis] / (2.0 * step);
      const double tolerance = 0.005 + 0.02 * std::abs(derivative);
      const double a = jacobian.data[axis * dof + joint];
      const double b = jacobian.data[joint * 6 + axis];
      spatial_error = std::max(spatial_error, std::abs(a - derivative));
      transposed_error = std::max(transposed_error, std::abs(b - derivative));
      spatial = spatial && std::isfinite(a) && std::abs(a - derivative) <= tolerance;
      transposed = transposed && std::isfinite(b) && std::abs(b - derivative) <= tolerance;
    }
  }
  if (logger_) logger_->event("mit_jacobian_check",
      QString("spatial_rows=%1;joint_rows=%2;spatial_max_error=%3;joint_max_error=%4")
          .arg(spatial).arg(transposed).arg(spatial_error, 0, 'g', 10).arg(transposed_error, 0, 'g', 10));
  if (spatial == transposed) {
    *reason = "MIT Jacobian verification failed/ambiguous: frame, units or layout do not match base-frame FK. No MIT frame sent.";
    return false;
  }
  mit_controller_.setJacobianLayout(spatial ? MitHandleController::JacobianLayout::SpatialRows
                                          : MitHandleController::JacobianLayout::JointRows);
  return true;
}

bool ArmWorker::validateMitConfiguration(QString* reason) const {
  const auto config_error = validateConfig(config_);
  if (!config_error.isEmpty()) { *reason = config_error; return false; }
  const auto mit = config_.value("mit").toObject();
  const double period_ms = mit.value("period_ms").toDouble(-1.0);
  const double joint_kp = mit.value("joint_kp").toDouble(-1.0);
  const double joint_kd = mit.value("joint_kd").toDouble(-1.0);
  if (!std::isfinite(period_ms) || period_ms <= 0.0 || !std::isfinite(joint_kp) || joint_kp < 0.0 ||
      !std::isfinite(joint_kd) || joint_kd < 0.0 || mit_max_failures_ < 1 || mit_watchdog_ms_ <= 0 ||
      mit_watchdog_ms_ < period_ms) {
    *reason = "Invalid MIT configuration: use positive period/watchdog/failure values and non-negative gains.";
    return false;
  }
  return true;
}

void ArmWorker::stopMitHandle(const QString& reason) {
  abortFrictionBatch(reason);
  if (friction_probe_.active && logger_) logger_->event("friction_probe_cancelled",reason);
  friction_probe_.cancel();
  mit_startup_hold_.reset();
  if (!mit_active_) return;
  mit_timer_.stop();
  mit_active_ = false;
  const bool exited = leaveLowMode("mit_handle");
  mit_controller_.reset();
  mit_last_accepted_position_.clear();
  modes_.exitHandle();
  snapshot_.application_mode = modes_.mode();
  if (logger_) logger_->event("mit_handle_stop", reason + QString(";exit_requests_accepted=%1;verification=pending").arg(exited));
}

void ArmWorker::fail(const QString& reason) {
  stopMitHandle(reason);
  snapshot_.error = reason;
  snapshot_.safety = SafetyState::Fault;
  modes_.exitHandle();
  snapshot_.application_mode = modes_.mode();
  emit message(reason);
  if (logger_) logger_->event("fault", reason);
  publish();
}

void ArmWorker::publish() {
  snapshot_.friction_batch_active=friction_batch_.active();
  snapshot_.friction_batch_completed=friction_batch_.index();
  snapshot_.low_session = low_mode_active_;
  snapshot_.exit_pending = exit_pending_;
  snapshot_.mit_running = mit_active_;
  snapshot_.gravity_test = mit_active_ && gravity_test_;
  snapshot_.mit_startup_holding = mit_active_ && mit_startup_hold_.active();
  if (!mit_active_) snapshot_.gravity_only_unlimited = false;
  snapshot_.log_ready = logger_ && logger_->isReady();
  snapshot_.session_directory = logger_ ? logger_->sessionDirectory() : QString();
  if (logger_) logger_->state(snapshot_);
  emit snapshotUpdated(snapshot_);
}
