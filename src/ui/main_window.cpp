#include "ui/main_window.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

#include "arm/arm_worker.hpp"
#include "control/handle_mode_manager.hpp"

namespace {
QString formatVector(const std::vector<double>& values) {
  QStringList parts;
  for (double value : values) parts << QString::number(value, 'f', 3);
  return "[" + parts.join(", ") + "]";
}
QString formatPose(const std::array<double, 7>& values) {
  QStringList parts;
  for (double value : values) parts << QString::number(value, 'f', 4);
  return "[" + parts.join(", ") + "]";
}
QDoubleSpinBox* factorBox(double value) {
  auto* box = new QDoubleSpinBox();
  box->setRange(0.0, 2.0);
  box->setSingleStep(0.05);
  box->setDecimals(2);
  box->setValue(value);
  return box;
}
QDoubleSpinBox* numberBox(double minimum, double maximum, double step, double value) {
  auto* box = new QDoubleSpinBox();
  box->setRange(minimum, maximum);
  box->setSingleStep(step);
  box->setDecimals(3);
  box->setValue(value);
  return box;
}
}  // namespace

MainWindow::MainWindow(const QJsonObject& config, QWidget* parent) : QMainWindow(parent) {
  setWindowTitle("A3-T Handle Console");
  auto* root = new QWidget(this);
  auto* root_layout = new QVBoxLayout(root);
  auto* tabs = new QTabWidget();
  root_layout->addWidget(tabs);

  // Connection and safety tab -------------------------------------------------
  auto* overview = new QWidget();
  auto* overview_layout = new QVBoxLayout(overview);
  auto* status_group = new QGroupBox("Controller state");
  auto* status_form = new QFormLayout(status_group);
  ip_edit_ = new QLineEdit(config.value("controller_ip").toString("10.42.0.101"));
  connection_label_ = new QLabel("Disconnected");
  servo_label_ = new QLabel("Disabled");
  safety_label_ = new QLabel("Normal");
  vendor_fsm_label_ = new QLabel("0");
  target_label_ = new QLabel("Physical robot");
  application_mode_label_ = new QLabel("Monitor");
  joints_label_ = new QLabel("-");
  pose_label_ = new QLabel("-");
  status_form->addRow("Controller IP", ip_edit_);
  status_form->addRow("Connection", connection_label_);
  status_form->addRow("Servo", servo_label_);
  status_form->addRow("Safety", safety_label_);
  status_form->addRow("Vendor FSM state", vendor_fsm_label_);
  status_form->addRow("Target", target_label_);
  status_form->addRow("Application mode", application_mode_label_);
  status_form->addRow("Joint position (rad)", joints_label_);
  status_form->addRow("Cartesian pose", pose_label_);
  overview_layout->addWidget(status_group);

  auto* connection_group = new QGroupBox("Connection and safety");
  auto* connection_layout = new QHBoxLayout(connection_group);
  auto* connect_button = new QPushButton("Connect");
  auto* disconnect_button = new QPushButton("Disconnect");
  auto* ready_button = new QPushButton("Vendor Ready");
  auto* disable_button = new QPushButton("Disable Servo");
  auto* estop_button = new QPushButton("Emergency Stop");
  estop_button->setStyleSheet("font-weight: bold; color: #a00;");
  for (auto* button : {connect_button, disconnect_button, ready_button, disable_button, estop_button}) connection_layout->addWidget(button);
  overview_layout->addWidget(connection_group);

  auto* safety_group = new QGroupBox("Vendor safety settings");
  auto* safety_layout = new QFormLayout(safety_group);
  speed_level_input_ = new QDoubleSpinBox();
  speed_level_input_->setRange(0.0, 10.0);
  speed_level_input_->setSingleStep(0.5);
  speed_level_input_->setValue(3.0);
  auto* speed_apply = new QPushButton("Apply speed level");
  auto* collision_enabled = new QCheckBox("Enable collision protection");
  collision_enabled->setChecked(true);
  collision_sensitivity_input_ = new QComboBox();
  collision_sensitivity_input_->addItems({"0 (highest)", "1", "2"});
  auto* collision_apply = new QPushButton("Apply collision settings");
  safety_layout->addRow("Speed level [0, 10]", speed_level_input_);
  safety_layout->addRow("", speed_apply);
  safety_layout->addRow("Collision", collision_enabled);
  safety_layout->addRow("Sensitivity", collision_sensitivity_input_);
  safety_layout->addRow("", collision_apply);
  overview_layout->addWidget(safety_group);
  overview_layout->addStretch();
  tabs->addTab(overview, "Connection & Safety");

  // Drag and vendor teach tab -------------------------------------------------
  auto* drag_tab = new QWidget();
  auto* drag_layout = new QVBoxLayout(drag_tab);
  auto* drag_group = new QGroupBox("Vendor Drag mode");
  auto* drag_form = new QFormLayout(drag_group);
  const auto drag_config = config.value("drag").toObject();
  torque_factor_input_ = factorBox(drag_config.value("torque_factor").toDouble(1.0));
  friction_factor_input_ = factorBox(drag_config.value("friction_compensation_factor").toDouble(0.8));
  auto* drag_enter = new QPushButton("Enter Vendor Drag");
  auto* drag_apply = new QPushButton("Apply Drag Parameters");
  auto* drag_exit = new QPushButton("Return to Idle");
  drag_form->addRow("Torque factor [0, 2]", torque_factor_input_);
  drag_form->addRow("Friction compensation [0, 2]", friction_factor_input_);
  drag_form->addRow("", drag_enter);
  drag_form->addRow("", drag_apply);
  drag_form->addRow("", drag_exit);
  drag_layout->addWidget(drag_group);

  auto* teach_group = new QGroupBox("Vendor teach trajectory");
  auto* teach_form = new QFormLayout(teach_group);
  teach_name_edit_ = new QLineEdit();
  teach_name_edit_->setPlaceholderText("Trajectory name");
  teach_list_ = new QComboBox();
  auto* teach_start = new QPushButton("Start recording");
  auto* teach_stop = new QPushButton("Stop recording");
  auto* teach_refresh = new QPushButton("Refresh list");
  auto* teach_replay = new QPushButton("Replay selected");
  teach_form->addRow("Name", teach_name_edit_);
  teach_form->addRow("", teach_start);
  teach_form->addRow("", teach_stop);
  teach_form->addRow("Recorded trajectories", teach_list_);
  teach_form->addRow("", teach_refresh);
  teach_form->addRow("", teach_replay);
  drag_layout->addWidget(teach_group);
  drag_layout->addStretch();
  tabs->addTab(drag_tab, "Drag & Teach");

  // Handle V0 is a safe dry-run implementation of the final control math. It uses
  // vendor Drag physically and only logs the calculated virtual wrench.
  auto* handle_tab = new QWidget();
  auto* handle_layout = new QVBoxLayout(handle_tab);
  auto* handle_notice = new QLabel("Handle V0 captures a Cartesian center and computes 6-DOF relative motion plus virtual spring-damper wrench. It uses Vendor Drag physically and does not send MIT torque yet.");
  handle_notice->setWordWrap(true);
  handle_layout->addWidget(handle_notice);
  auto* handle_group = new QGroupBox("6-DOF Cartesian impedance profile");
  auto* handle_form = new QFormLayout(handle_group);
  const auto handle_config = config.value("handle").toObject();
  auto* position_scale = numberBox(0.01, 10.0, 0.1, handle_config.value("position_scale").toDouble(1.0));
  auto* rotation_scale = numberBox(0.01, 10.0, 0.1, handle_config.value("rotation_scale").toDouble(1.0));
  auto* position_stiffness = numberBox(0.0, 200.0, 1.0, handle_config.value("position_stiffness").toDouble(20.0));
  auto* position_damping = numberBox(0.0, 100.0, 0.5, handle_config.value("position_damping").toDouble(4.0));
  auto* rotation_stiffness = numberBox(0.0, 20.0, 0.1, handle_config.value("rotation_stiffness").toDouble(1.0));
  auto* rotation_damping = numberBox(0.0, 10.0, 0.05, handle_config.value("rotation_damping").toDouble(0.1));
  auto* capture_center = new QPushButton("Capture center pose");
  auto* apply_handle = new QPushButton("Apply Handle parameters");
  auto* enter_handle = new QPushButton("Enter Handle V0");
  auto* enter_mit = new QPushButton("Enter Handle MIT (hardware locked)");
  auto* clutch = new QPushButton("Clutch / reset center");
  handle_center_label_ = new QLabel("Not captured");
  handle_command_label_ = new QLabel("-");
  virtual_wrench_label_ = new QLabel("-");
  handle_form->addRow("Position scale", position_scale);
  handle_form->addRow("Rotation scale", rotation_scale);
  handle_form->addRow("Position stiffness [N/m]", position_stiffness);
  handle_form->addRow("Position damping [N·s/m]", position_damping);
  handle_form->addRow("Rotation stiffness [N·m/rad]", rotation_stiffness);
  handle_form->addRow("Rotation damping [N·m·s/rad]", rotation_damping);
  handle_form->addRow("", capture_center);
  handle_form->addRow("", apply_handle);
  handle_form->addRow("", enter_handle);
  handle_form->addRow("", enter_mit);
  handle_form->addRow("", clutch);
  handle_form->addRow("Center", handle_center_label_);
  handle_form->addRow("6-DOF command", handle_command_label_);
  handle_form->addRow("Virtual wrench", virtual_wrench_label_);
  handle_layout->addWidget(handle_group);
  handle_layout->addStretch();
  tabs->addTab(handle_tab, "Handle V0");

  // Tracking layout -----------------------------------------------------------
  auto* tracking_tab = new QWidget();
  auto* tracking_layout = new QVBoxLayout(tracking_tab);
  auto* tracking_notice = new QLabel("Tracking will replay a validated trajectory or accept a live pose source through track_joint() / track_pose(). No external source or trajectory file format has been selected yet.");
  tracking_notice->setWordWrap(true);
  auto* tracking_group = new QGroupBox("Tracking controls");
  auto* tracking_form = new QFormLayout(tracking_group);
  auto* source = new QLineEdit("No trajectory source configured");
  auto* tracking_start = new QPushButton("Start tracking");
  auto* tracking_stop = new QPushButton("Stop tracking");
  source->setEnabled(false);
  tracking_start->setEnabled(false);
  tracking_stop->setEnabled(false);
  tracking_form->addRow("Source", source);
  tracking_form->addRow("", tracking_start);
  tracking_form->addRow("", tracking_stop);
  tracking_layout->addWidget(tracking_notice);
  tracking_layout->addWidget(tracking_group);
  tracking_layout->addStretch();
  tabs->addTab(tracking_tab, "Tracking (planned)");

  // Diagnostics ---------------------------------------------------------------
  auto* diagnostics_tab = new QWidget();
  auto* diagnostics_layout = new QVBoxLayout(diagnostics_tab);
  message_label_ = new QLabel("Ready. No MIT command is implemented.");
  message_label_->setWordWrap(true);
  event_log_ = new QPlainTextEdit();
  event_log_->setReadOnly(true);
  diagnostics_layout->addWidget(message_label_);
  diagnostics_layout->addWidget(event_log_);
  tabs->addTab(diagnostics_tab, "Diagnostics");
  setCentralWidget(root);

  qRegisterMetaType<ArmSnapshot>("ArmSnapshot");
  worker_ = new ArmWorker(config);
  worker_->moveToThread(&worker_thread_);
  connect(connect_button, &QPushButton::clicked, this, [this] {
    QMetaObject::invokeMethod(worker_, "connectRobot", Qt::QueuedConnection, Q_ARG(QString, ip_edit_->text()));
  });
  connect(disconnect_button, &QPushButton::clicked, worker_, &ArmWorker::disconnectRobot, Qt::QueuedConnection);
  connect(ready_button, &QPushButton::clicked, worker_, &ArmWorker::setReady, Qt::QueuedConnection);
  connect(disable_button, &QPushButton::clicked, worker_, &ArmWorker::disableServo, Qt::QueuedConnection);
  connect(estop_button, &QPushButton::clicked, worker_, &ArmWorker::emergencyStop, Qt::QueuedConnection);
  connect(speed_apply, &QPushButton::clicked, this, [this] {
    QMetaObject::invokeMethod(worker_, "setSpeedLevel", Qt::QueuedConnection, Q_ARG(double, speed_level_input_->value()));
  });
  connect(collision_apply, &QPushButton::clicked, this, [this, collision_enabled] {
    QMetaObject::invokeMethod(worker_, "setCollisionProtection", Qt::QueuedConnection,
                              Q_ARG(bool, collision_enabled->isChecked()),
                              Q_ARG(int, collision_sensitivity_input_->currentIndex()));
  });
  connect(drag_enter, &QPushButton::clicked, worker_, &ArmWorker::enterDragMode, Qt::QueuedConnection);
  connect(drag_apply, &QPushButton::clicked, this, [this] {
    QMetaObject::invokeMethod(worker_, "applyDragParameters", Qt::QueuedConnection,
                              Q_ARG(double, torque_factor_input_->value()),
                              Q_ARG(double, friction_factor_input_->value()));
  });
  connect(drag_exit, &QPushButton::clicked, worker_, &ArmWorker::exitHandleMode, Qt::QueuedConnection);
  connect(teach_start, &QPushButton::clicked, this, [this] {
    QMetaObject::invokeMethod(worker_, "startTeach", Qt::QueuedConnection, Q_ARG(QString, teach_name_edit_->text()));
  });
  connect(teach_stop, &QPushButton::clicked, worker_, &ArmWorker::stopTeach, Qt::QueuedConnection);
  connect(teach_refresh, &QPushButton::clicked, worker_, &ArmWorker::refreshTeachList, Qt::QueuedConnection);
  connect(teach_replay, &QPushButton::clicked, this, [this] {
    QMetaObject::invokeMethod(worker_, "replayTeach", Qt::QueuedConnection, Q_ARG(QString, teach_list_->currentText()));
  });
  connect(capture_center, &QPushButton::clicked, worker_, &ArmWorker::captureHandleCenter, Qt::QueuedConnection);
  connect(clutch, &QPushButton::clicked, worker_, &ArmWorker::captureHandleCenter, Qt::QueuedConnection);
  connect(apply_handle, &QPushButton::clicked, this, [this, position_scale, rotation_scale,
                                                        position_stiffness, position_damping,
                                                        rotation_stiffness, rotation_damping] {
    QMetaObject::invokeMethod(worker_, "applyHandleParameters", Qt::QueuedConnection,
                              Q_ARG(double, position_scale->value()), Q_ARG(double, rotation_scale->value()),
                              Q_ARG(double, position_stiffness->value()), Q_ARG(double, position_damping->value()),
                              Q_ARG(double, rotation_stiffness->value()), Q_ARG(double, rotation_damping->value()));
  });
  connect(enter_handle, &QPushButton::clicked, worker_, &ArmWorker::enterHandleMode, Qt::QueuedConnection);
  connect(enter_mit, &QPushButton::clicked, worker_, &ArmWorker::startMitHandle, Qt::QueuedConnection);
  connect(worker_, &ArmWorker::snapshotUpdated, this, &MainWindow::updateSnapshot);
  connect(worker_, &ArmWorker::message, this, &MainWindow::showMessage);
  connect(worker_, &ArmWorker::teachListUpdated, this, &MainWindow::updateTeachList);
  worker_thread_.start();
}

MainWindow::~MainWindow() {
  QMetaObject::invokeMethod(worker_, "disconnectRobot", Qt::BlockingQueuedConnection);
  worker_thread_.quit();
  worker_thread_.wait();
  delete worker_;
}

void MainWindow::updateSnapshot(const ArmSnapshot& snapshot) {
  connection_label_->setText(connectionName(snapshot.connection));
  servo_label_->setText(servoName(snapshot.servo));
  safety_label_->setText(safetyName(snapshot.safety));
  vendor_fsm_label_->setText(QString::number(snapshot.vendor_fsm_state));
  target_label_->setText(snapshot.vendor_debug_mode ? "Simulation / debug" : "Physical robot");
  application_mode_label_->setText(applicationModeName(snapshot.application_mode));
  joints_label_->setText(formatVector(snapshot.joint_position));
  pose_label_->setText(formatPose(snapshot.cartesian_pose));
  handle_center_label_->setText(snapshot.handle_center_captured ? "Captured" : "Not captured");
  handle_command_label_->setText(formatVector({snapshot.handle_command.begin(), snapshot.handle_command.end()}));
  virtual_wrench_label_->setText(formatVector({snapshot.virtual_wrench.begin(), snapshot.virtual_wrench.end()}));
  if (!snapshot.error.isEmpty()) showMessage(snapshot.error);
}

void MainWindow::showMessage(const QString& message) {
  message_label_->setText(message);
  event_log_->appendPlainText(message);
}

void MainWindow::updateTeachList(const QStringList& names) {
  teach_list_->clear();
  teach_list_->addItems(names);
}
