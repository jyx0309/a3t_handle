#pragma once

#include <QFile>
#include <QString>
#include <QTextStream>
#include <QElapsedTimer>
#include <QJsonObject>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "arm/arm_types.hpp"
#include "math/handle_controller.hpp"

// Producer methods run on ArmWorker; serialization and file IO run on writer_.
class SessionLogger final {
 public:
  explicit SessionLogger(const QString& root_directory);
  ~SessionLogger();

  bool isReady() const { return ready_; }
  QString sessionDirectory() const { return session_directory_; }
  void event(const QString& action, const QString& detail = {});
  void command(const QString& name, const QString& detail = {});
  void state(const ArmSnapshot& snapshot);
  void performance(const QString& name, double milliseconds);
  void handle(const HandleOutput& output);
  void diagnostic(QJsonObject record);

 private:
  static QString timestamp();
  static QString csv(const QString& value);
  static QString vectorText(const std::vector<double>& values);
  static QString poseText(const std::array<double, 7>& pose);
  enum Destination { Events, Commands, States, Performance, Handle, Diagnostic, Count };
  struct Record { Destination destination; QString line; QJsonObject json; };
  void write(Destination destination, const QString& line);
  void enqueue(Record record);
  void runWriter();

  std::atomic<bool> ready_{false};
  QElapsedTimer diagnostic_clock_;
  QString session_directory_;
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<Record> queue_;
  bool initialized_{false}, stopping_{false};
  static constexpr size_t queue_capacity_ = 8192;
  std::thread writer_;
};
