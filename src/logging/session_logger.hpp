#pragma once

#include <QFile>
#include <QString>
#include <QTextStream>

#include "arm/arm_types.hpp"
#include "math/handle_controller.hpp"

// All methods are called from ArmWorker's single control thread.
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

 private:
  static QString timestamp();
  static QString csv(const QString& value);
  static QString vectorText(const std::vector<double>& values);
  static QString poseText(const std::array<double, 7>& pose);
  void write(QTextStream& stream, const QString& line);

  bool ready_{false};
  QString session_directory_;
  QFile events_file_;
  QFile commands_file_;
  QFile states_file_;
  QFile performance_file_;
  QFile handle_file_;
  QTextStream events_;
  QTextStream commands_;
  QTextStream states_;
  QTextStream performance_;
  QTextStream handle_;
};
