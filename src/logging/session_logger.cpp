#include "logging/session_logger.hpp"

#include <QDateTime>
#include <QDir>

#include "control/handle_mode_manager.hpp"

SessionLogger::SessionLogger(const QString& root_directory) {
  const QString root = QDir(root_directory).absolutePath();
  session_directory_ = QDir(root).filePath(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz"));
  if (!QDir().mkpath(session_directory_)) return;

  events_file_.setFileName(QDir(session_directory_).filePath("events.csv"));
  commands_file_.setFileName(QDir(session_directory_).filePath("commands.csv"));
  states_file_.setFileName(QDir(session_directory_).filePath("states.csv"));
  performance_file_.setFileName(QDir(session_directory_).filePath("performance.csv"));
  handle_file_.setFileName(QDir(session_directory_).filePath("handle.csv"));
  if (!events_file_.open(QIODevice::WriteOnly | QIODevice::Text) ||
      !commands_file_.open(QIODevice::WriteOnly | QIODevice::Text) ||
      !states_file_.open(QIODevice::WriteOnly | QIODevice::Text) ||
      !performance_file_.open(QIODevice::WriteOnly | QIODevice::Text) ||
      !handle_file_.open(QIODevice::WriteOnly | QIODevice::Text)) return;

  events_.setDevice(&events_file_);
  commands_.setDevice(&commands_file_);
  states_.setDevice(&states_file_);
  performance_.setDevice(&performance_file_);
  handle_.setDevice(&handle_file_);
  events_ << "timestamp,action,detail\n";
  commands_ << "timestamp,name,detail\n";
  states_ << "timestamp,connection,servo,safety,controller_state,vendor_fsm,debug,application_mode,joint_position,cartesian_pose,error\n";
  performance_ << "timestamp,name,milliseconds\n";
  handle_ << "timestamp,dt,raw_error,filtered_error,command,virtual_wrench\n";
  ready_ = true;
  event("session_started", session_directory_);
}

SessionLogger::~SessionLogger() {
  if (ready_) event("session_finished");
}

QString SessionLogger::timestamp() { return QDateTime::currentDateTime().toString(Qt::ISODateWithMs); }

QString SessionLogger::csv(const QString& value) {
  QString escaped = value;
  escaped.replace('"', "\"\"");
  return '"' + escaped + '"';
}

QString SessionLogger::vectorText(const std::vector<double>& values) {
  QStringList fields;
  for (double value : values) fields << QString::number(value, 'g', 12);
  return fields.join(';');
}

QString SessionLogger::poseText(const std::array<double, 7>& pose) {
  QStringList fields;
  for (double value : pose) fields << QString::number(value, 'g', 12);
  return fields.join(';');
}

void SessionLogger::write(QTextStream& stream, const QString& line) {
  if (!ready_) return;
  stream << line << '\n';
  stream.flush();
}

void SessionLogger::event(const QString& action, const QString& detail) {
  write(events_, timestamp() + ',' + csv(action) + ',' + csv(detail));
}

void SessionLogger::command(const QString& name, const QString& detail) {
  write(commands_, timestamp() + ',' + csv(name) + ',' + csv(detail));
}

void SessionLogger::state(const ArmSnapshot& snapshot) {
  write(states_, timestamp() + ',' + csv(connectionName(snapshot.connection)) + ',' +
                  csv(servoName(snapshot.servo)) + ',' + csv(safetyName(snapshot.safety)) + ',' +
                  QString::number(snapshot.controller_state) + ',' + QString::number(snapshot.vendor_fsm_state) + ',' +
                  QString::number(snapshot.vendor_debug_mode) + ',' + csv(applicationModeName(snapshot.application_mode)) + ',' +
                  csv(vectorText(snapshot.joint_position)) + ',' + csv(poseText(snapshot.cartesian_pose)) + ',' + csv(snapshot.error));
}

void SessionLogger::performance(const QString& name, double milliseconds) {
  write(performance_, timestamp() + ',' + csv(name) + ',' + QString::number(milliseconds, 'f', 3));
}

void SessionLogger::handle(const HandleOutput& output) {
  if (!output.valid) return;
  write(handle_, timestamp() + ',' + QString::number(output.dt, 'f', 6) + ',' +
                 csv(vectorText({output.raw_error.begin(), output.raw_error.end()})) + ',' +
                 csv(vectorText({output.filtered_error.begin(), output.filtered_error.end()})) + ',' +
                 csv(vectorText({output.command.begin(), output.command.end()})) + ',' +
                 csv(vectorText({output.virtual_wrench.begin(), output.virtual_wrench.end()})));
}
