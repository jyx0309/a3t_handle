#include "logging/session_logger.hpp"

#include <QDateTime>
#include <QDir>
#include <QJsonDocument>
#include <chrono>

#include "control/handle_mode_manager.hpp"

SessionLogger::SessionLogger(const QString& root_directory) {
  const QString root = QDir(root_directory).absolutePath();
  session_directory_ = QDir(root).filePath(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz"));
  if (!QDir().mkpath(session_directory_)) return;
  diagnostic_clock_.start();
  writer_ = std::thread([this] { runWriter(); });
  {
    std::unique_lock<std::mutex> lock(mutex_);
    wake_.wait(lock, [this] { return initialized_; });
  }
  event("session_started", session_directory_);
}

SessionLogger::~SessionLogger() {
  if (ready_) event("session_finished");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  if (writer_.joinable()) writer_.join();
}

void SessionLogger::enqueue(Record record) {
  if (!ready_) return;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Bounded backlog: fail closed instead of blocking control on storage or
    // silently losing records. ArmWorker checks isReady each control cycle.
    if (stopping_ || queue_.size() >= queue_capacity_) { ready_ = false; return; }
    queue_.push_back(std::move(record));
  }
  wake_.notify_one();
}

void SessionLogger::runWriter() {
  // QFile objects are created, used and destroyed only on this thread.
  QFile files[Count];
  const char* names[Count] = {"events.csv", "commands.csv", "states.csv", "performance.csv", "handle.csv", "diagnostics.jsonl"};
  const QByteArray headers[Count] = {
    "timestamp,action,detail\n", "timestamp,name,detail\n",
    "timestamp,connection,servo,safety,controller_state,vendor_fsm,debug,application_mode,joint_position,cartesian_pose,error\n",
    "timestamp,name,milliseconds\n", "timestamp,dt,raw_error,filtered_error,command,virtual_wrench\n", ""};
  bool ok = true;
  for (int i = 0; i < Count; ++i) {
    files[i].setFileName(QDir(session_directory_).filePath(names[i]));
    ok = files[i].open(QIODevice::WriteOnly | QIODevice::Text) && ok;
    if (ok) ok = files[i].write(headers[i]) == headers[i].size() && files[i].flush();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ready_ = ok;
    initialized_ = true;
  }
  wake_.notify_one();
  if (!ok) return;
  QElapsedTimer flush_clock;
  flush_clock.start();
  for (;;) {
    std::deque<Record> batch;
    bool done = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(100), [this] { return stopping_ || !queue_.empty(); });
      // Do not hold the queue lock during encoding or file IO.
      for (int i = 0; i < 256 && !queue_.empty(); ++i) {
        batch.push_back(std::move(queue_.front()));
        queue_.pop_front();
      }
      done = stopping_ && queue_.empty();
    }
    bool urgent = false;
    for (const auto& record : batch) {
      const QByteArray bytes = record.destination == Diagnostic ?
          QJsonDocument(record.json).toJson(QJsonDocument::Compact) + '\n' : record.line.toUtf8() + '\n';
      if (files[record.destination].write(bytes) != bytes.size()) ok = false;
      urgent = urgent || record.destination == Events || record.destination == Commands;
    }
    if (urgent || done || flush_clock.elapsed() >= 100) {
      for (auto& file : files) if (!file.flush()) ok = false;
      flush_clock.restart();
    }
    if (!ok) { ready_ = false; return; }
    if (done) return;
  }
}

void SessionLogger::diagnostic(QJsonObject record) {
  if (!ready_) return;
  record["timestamp"] = timestamp();
  record["monotonic_ms"] = diagnostic_clock_.nsecsElapsed() / 1e6;
  record["schema"] = 1;
  enqueue({Diagnostic, {}, std::move(record)});
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

void SessionLogger::write(Destination destination, const QString& line) {
  enqueue({destination, line, {}});
}

void SessionLogger::event(const QString& action, const QString& detail) {
  write(Events, timestamp() + ',' + csv(action) + ',' + csv(detail));
  diagnostic({{"type", "event"}, {"action", action}, {"detail", detail}});
}

void SessionLogger::command(const QString& name, const QString& detail) {
  write(Commands, timestamp() + ',' + csv(name) + ',' + csv(detail));
}

void SessionLogger::state(const ArmSnapshot& snapshot) {
  write(States, timestamp() + ',' + csv(connectionName(snapshot.connection)) + ',' +
                  csv(servoName(snapshot.servo)) + ',' + csv(safetyName(snapshot.safety)) + ',' +
                  QString::number(snapshot.controller_state) + ',' + QString::number(snapshot.vendor_fsm_state) + ',' +
                  QString::number(snapshot.vendor_debug_mode) + ',' + csv(applicationModeName(snapshot.application_mode)) + ',' +
                  csv(vectorText(snapshot.joint_position)) + ',' + csv(poseText(snapshot.cartesian_pose)) + ',' + csv(snapshot.error));
}

void SessionLogger::performance(const QString& name, double milliseconds) {
  write(Performance, timestamp() + ',' + csv(name) + ',' + QString::number(milliseconds, 'f', 3));
}

void SessionLogger::handle(const HandleOutput& output) {
  if (!output.valid) return;
  write(Handle, timestamp() + ',' + QString::number(output.dt, 'f', 6) + ',' +
                 csv(vectorText({output.raw_error.begin(), output.raw_error.end()})) + ',' +
                 csv(vectorText({output.filtered_error.begin(), output.filtered_error.end()})) + ',' +
                 csv(vectorText({output.command.begin(), output.command.end()})) + ',' +
                 csv(vectorText({output.virtual_wrench.begin(), output.virtual_wrench.end()})));
}
