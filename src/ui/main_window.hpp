#pragma once
#include <QJsonObject>
#include <QMainWindow>
#include <QThread>
#include "arm/arm_types.hpp"
class ArmWorker;
class QLabel;
class QPlainTextEdit;
class QCloseEvent;

class MainWindow final : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(const QJsonObject& config, QWidget* parent = nullptr);
  ~MainWindow() override;
  bool enableLocalControl(const QString& path, const QString& ip);
 protected:
  void closeEvent(QCloseEvent* event) override;
 private:
  void showMessage(const QString& message);
  QThread worker_thread_;
  ArmWorker* worker_{nullptr};
  ArmSnapshot snapshot_;
  QLabel* banner_{nullptr};
  QLabel* values_{nullptr};
  QLabel* diagnostics_{nullptr};
  QPlainTextEdit* event_log_{nullptr};
  QString last_message_;
};
