#pragma once

#include <QJsonObject>
#include <QMainWindow>
#include <QThread>

#include "arm/arm_types.hpp"

class ArmWorker;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;

class MainWindow final : public QMainWindow {
  Q_OBJECT
 public:
  explicit MainWindow(const QJsonObject& config, QWidget* parent = nullptr);
  ~MainWindow() override;

 private slots:
  void updateSnapshot(const ArmSnapshot& snapshot);
  void showMessage(const QString& message);
  void updateTeachList(const QStringList& names);

 private:
  QThread worker_thread_;
  ArmWorker* worker_{nullptr};
  QLineEdit* ip_edit_{nullptr};
  QLineEdit* teach_name_edit_{nullptr};
  QComboBox* teach_list_{nullptr};
  QDoubleSpinBox* torque_factor_input_{nullptr};
  QDoubleSpinBox* friction_factor_input_{nullptr};
  QDoubleSpinBox* speed_level_input_{nullptr};
  QComboBox* collision_sensitivity_input_{nullptr};
  QLabel* connection_label_{nullptr};
  QLabel* servo_label_{nullptr};
  QLabel* safety_label_{nullptr};
  QLabel* vendor_fsm_label_{nullptr};
  QLabel* target_label_{nullptr};
  QLabel* application_mode_label_{nullptr};
  QLabel* joints_label_{nullptr};
  QLabel* pose_label_{nullptr};
  QLabel* message_label_{nullptr};
  QLabel* handle_center_label_{nullptr};
  QLabel* handle_command_label_{nullptr};
  QLabel* virtual_wrench_label_{nullptr};
  QPlainTextEdit* event_log_{nullptr};
};
