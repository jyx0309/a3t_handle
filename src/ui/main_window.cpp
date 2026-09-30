#include "ui/main_window.hpp"
#include "arm/arm_worker.hpp"
#include "arm/mit_entry.hpp"
#include "ui/assist_curve.hpp"
#include "config/near_assist.hpp"
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDesktopServices>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QTabWidget>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QJsonArray>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QUrl>
#include <QVBoxLayout>

namespace {
template <typename T> QString vectorText(const T& data) {
  QStringList text;
  for (double v : data) text << QString::number(v, 'f', 4);
  return "[" + text.join(", ") + "]";
}
}

MainWindow::MainWindow(const QJsonObject& config, QWidget* parent) : QMainWindow(parent) {
  qRegisterMetaType<ArmSnapshot>("ArmSnapshot");
  setWindowTitle("A3-T · 底层 MIT 手柄");
  auto* root = new QWidget(this);
  setCentralWidget(root);
  auto* layout = new QVBoxLayout(root);
  banner_ = new QLabel("未连接 · 不自动启动机械臂");
  banner_->setStyleSheet("font-size:18px; font-weight:600; padding:12px; background:#e8eff4;");
  layout->addWidget(banner_);
  auto* controls = new QHBoxLayout;
  auto* ip = new QLineEdit(config.value("controller_ip").toString());
  auto* connect_button = new QPushButton("连接");
  auto* disconnect = new QPushButton("断开");
  auto* ready = new QPushButton("复位并使能");
  auto* disable = new QPushButton("高层下使能");
  auto* emergency = new QPushButton("软件急停（实体急停优先）");
  emergency->setStyleSheet("background:#b91c1c;color:white;padding:8px;");
  controls->addWidget(ip); controls->addWidget(connect_button); controls->addWidget(disconnect);
  controls->addWidget(ready); controls->addWidget(disable); controls->addWidget(emergency);
  layout->addLayout(controls);
  auto* operations = new QHBoxLayout;
  auto* start = new QPushButton("进入底层手柄（缓存回接 → 中心）");
  auto* stop = new QPushButton("停止手柄并返回高层");
  auto* gravity = new QPushButton("纯重力测试（零刚度／零阻尼）");
  gravity->setObjectName("gravityTestButton");
  operations->addWidget(start); operations->addWidget(stop);
  operations->addWidget(gravity);
  layout->addLayout(operations);
  auto* entry_hint = new QLabel("进入手柄：请先连接设备");
  entry_hint->setWordWrap(true);
  layout->addWidget(entry_hint);
  auto* probe_row = new QHBoxLayout;
  auto* probe_joint = new QComboBox;
  for (int i=1;i<=6;++i) probe_joint->addItem(QString("J%1").arg(i));
  auto* probe_direction = new QComboBox;
  probe_direction->addItems({"正向 (+)","反向 (-)"});
  auto* probe = new QPushButton("当前姿态起动力矩测试（单次）");
  auto* batch = new QPushButton("一键辨识全部（36 次）");
  auto* batch_progress = new QLabel("批次未运行");
  probe->setObjectName("frictionProbeButton");
  probe_row->addWidget(probe_joint); probe_row->addWidget(probe_direction); probe_row->addWidget(probe);
  layout->addLayout(probe_row);
  layout->addWidget(batch); layout->addWidget(batch_progress);
  connect(batch,&QPushButton::clicked,this,[this] {
    if (QMessageBox::warning(this,"确认纯重力连续加力测试",
        "以当前实际姿态记录本轮测试起点，不修改手柄中心；依次测 J1–J6，每个关节正反各3次，共36次。\n"
        "全程保持低层重力补偿；检测到运动后快速撤力并加入受限阻尼，制动力归零且静止后再测下一项，不回中心。\n"
        "需全程看护，手离开机械臂，防坠和实体急停就绪；不要操作厂家上位机。\n"
        "J1–J4：0.2 N·m/s、最高5 N·m；J5/J6：0.05 N·m/s、最高0.3 N·m。未起动撤力稳定后继续。\n"
        "异常或漂移保护仍中止，不自动重试/清错/应用补偿。\n"
        "累计偏离本轮起点超过0.05 rad或无法稳定会中止；结束仍保持纯重力，不代表机械臂被固定。停止按钮可取消整批。确认开始？",
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)==QMessageBox::Yes)
      QMetaObject::invokeMethod(worker_,&ArmWorker::startFrictionBatch,Qt::QueuedConnection);
  });
  connect(probe,&QPushButton::clicked,this,[this,probe_joint,probe_direction] {
    if (QMessageBox::warning(this,"确认逐关节加力试验",
        "必须已处于稳定的纯重力状态，以当前姿态作为测试起点；测试中无人接触机械臂，现场防坠措施及实体急停就绪。\n"
        "先观察1秒；J1–J4：0.2 N·m/s、最高5 N·m；J5/J6：0.05 N·m/s、最高0.3 N·m。\n"
        "达到起动条件快速撤力（J5/J6启用提前检测），单方向受限制动并稳定后记录候选；起动后最长2秒。\n"
        "0.015 rad位移/0.3 rad/s速度/其他关节0.005 rad漂移保护不变。\n"
        "结束沿现有下使能返回高层流程，不自动回中心，不自动应用辨识结果。确认现场可安全退出？",
        QMessageBox::Yes|QMessageBox::No,QMessageBox::No)!=QMessageBox::Yes) return;
    const int joint=probe_joint->currentIndex(), direction=probe_direction->currentIndex()==0?1:-1;
    QMetaObject::invokeMethod(worker_,[this,joint,direction]{worker_->startFrictionProbe(joint,direction);},Qt::QueuedConnection);
  });
  auto* notice = new QLabel("进入会运动到缓存姿态及配置中心；请确认路径安全。退出后观察 5 秒，返回高层不等于下使能。\n"
                           "高层回零/拖动使用厂家上位机；两套界面不要同时发命令。修改参数需先停止手柄。");
  notice->setWordWrap(true); layout->addWidget(notice);
  const bool local_gravity=config.value("gravity").toObject().value("source").toString()=="urdf";
  layout->addWidget(new QLabel(local_gravity ?
      "重力来源：本地 URDF（基座 -Z；含模型夹爪，未额外加入摄像头）；SDK 只作日志对照。" :
      "重力来源：SDK。"));
  auto* middle = new QHBoxLayout;
  auto* tuning = new QGroupBox("手柄调参（停止后应用）");
  auto* form = new QFormLayout(tuning);
  const auto h = config.value("handle").toObject();
  auto add = [&](const QString& name, double value) {
    auto* box = new QDoubleSpinBox;
    box->setObjectName(name);
    box->setRange(0.0, 1e9); box->setDecimals(6); box->setSingleStep(0.01);
    box->setValue(value); return box;
  };
  auto* tabs = new QTabWidget;
  auto* cart = new QWidget;
  auto* joints = new QWidget;
  auto* cart_grid = new QGridLayout(cart);
  auto* joint_grid = new QGridLayout(joints);
  tabs->addTab(cart, "笛卡尔六方向");
  tabs->addTab(joints, "MIT 六关节");
  form->addRow(tabs);
  cart_grid->addWidget(new QLabel("基座方向"), 0, 0);
  cart_grid->addWidget(new QLabel("刚度 K"), 0, 1);
  cart_grid->addWidget(new QLabel("阻尼 D"), 0, 2);
  joint_grid->addWidget(new QLabel("关节"), 0, 0);
  joint_grid->addWidget(new QLabel("kp"), 0, 1);
  joint_grid->addWidget(new QLabel("kd"), 0, 2);
  std::array<QDoubleSpinBox*, 6> cart_k{}, cart_d{}, joint_k{}, joint_d{};
  const QStringList axes{"X", "Y", "Z", "Rx", "Ry", "Rz"};
  const auto mit = config.value("mit").toObject();
  for (int i = 0; i < 6; ++i) {
    const auto prefix = i < 3 ? QString("position") : QString("rotation");
    const double axis_scale = h.value(prefix + "_axis_scale").toArray()[i % 3].toDouble(1.0);
    const double k = h.contains("cartesian_stiffness") ? h.value("cartesian_stiffness").toArray()[i].toDouble() :
        axis_scale * h.value(prefix + "_stiffness").toDouble();
    const double d = h.contains("cartesian_damping") ? h.value("cartesian_damping").toArray()[i].toDouble() :
        axis_scale * h.value(prefix + "_damping").toDouble();
    cart_k[i] = add("cartesianK" + axes[i], k);
    cart_d[i] = add("cartesianD" + axes[i], d);
    joint_k[i] = add(QString("mitKp%1").arg(i+1), mit.contains("normal_joint_kp") ?
        mit.value("normal_joint_kp").toArray()[i].toDouble() : mit.value("joint_kp").toDouble());
    joint_d[i] = add(QString("mitKd%1").arg(i+1), mit.contains("normal_joint_kd") ?
        mit.value("normal_joint_kd").toArray()[i].toDouble() : mit.value("joint_kd").toDouble());
    cart_grid->addWidget(new QLabel(axes[i]), i+1, 0);
    cart_grid->addWidget(cart_k[i], i+1, 1); cart_grid->addWidget(cart_d[i], i+1, 2);
    joint_grid->addWidget(new QLabel(QString("J%1").arg(i+1)), i+1, 0);
    joint_grid->addWidget(joint_k[i], i+1, 1); joint_grid->addWidget(joint_d[i], i+1, 2);
  }
  auto* units = new QLabel("XYZ：N/m、N·s/m；Rx/Ry/Rz：N·m/rad、N·m·s/rad。\n数值直接生效，不再乘旧方向倍率；回正力、关节辅助与摩擦补偿仍单独叠加。");
  units->setWordWrap(true); cart_grid->addWidget(units, 7, 0, 1, 3);
  auto* mit_note = new QLabel("正常模式目标角随实测角更新，目标速度为零。\n修改 kp 不会把目标固定在中心；纯重力模式仍将 kp/kd 置零。");
  mit_note->setWordWrap(true); joint_grid->addWidget(mit_note, 7, 0, 1, 3);
  auto* apply = new QPushButton("应用（本次会话）");
  auto* assist_page = new QWidget;
  auto* assist_layout = new QVBoxLayout(assist_page);
  auto* assist_grid = new QGridLayout;
  assist_layout->addLayout(assist_grid);
  tabs->addTab(assist_page, "近中心回正辅助");
  const QStringList headers{"方向", "启用", "幅值 N / Nm", "内过渡 m / rad", "外边界 m / rad"};
  for (int c=0;c<headers.size();++c) assist_grid->addWidget(new QLabel(headers[c]),0,c);
  HandleParameters assist_defaults;
  loadNearAssist(h, assist_defaults);
  std::array<QCheckBox*,6> assist_enabled{};
  std::array<QDoubleSpinBox*,6> assist_amp{}, assist_width{}, assist_range{};
  for (int i=0;i<6;++i) {
    assist_enabled[i]=new QCheckBox;
    assist_enabled[i]->setObjectName("assistEnabled"+axes[i]);
    assist_enabled[i]->setChecked(assist_defaults.near_assist_enabled[i]);
    assist_amp[i]=add("assistAmplitude"+axes[i],assist_defaults.near_assist_amplitude[i]);
    assist_width[i]=add("assistTransition"+axes[i],assist_defaults.near_assist_transition[i]);
    assist_range[i]=add("assistRange"+axes[i],assist_defaults.near_assist_range[i]);
    assist_width[i]->setSingleStep(i<3?0.0001:0.001);
    assist_range[i]->setSingleStep(i<3?0.001:0.01);
    assist_grid->addWidget(new QLabel(axes[i]),i+1,0);
    assist_grid->addWidget(assist_enabled[i],i+1,1);
    assist_grid->addWidget(assist_amp[i],i+1,2);
    assist_grid->addWidget(assist_width[i],i+1,3);
    assist_grid->addWidget(assist_range[i],i+1,4);
  }
  auto* explanation=new QLabel("启用的方向替代旧 breakaway；未启用沿用旧回正项。启用且幅值为零可关闭该方向附加回正。\n外边界是距中心的绝对距离；最后 25% 可用范围平滑撤力。初值仅供试验，非已辨识摩擦。");
  explanation->setWordWrap(true); assist_layout->addWidget(explanation);
  auto* curve_axis=new QComboBox; curve_axis->addItems(axes); assist_layout->addWidget(curve_axis);
  auto* curve=new AssistCurve; assist_layout->addWidget(curve);
  auto redraw=[=] {
    const int i=curve_axis->currentIndex();
    curve->deadband=h.value(i<3?"position_deadband_m":"rotation_deadband_rad").toDouble();
    curve->width=assist_width[i]->value(); curve->range=assist_range[i]->value();
    curve->amplitude=assist_amp[i]->value(); curve->stiffness=cart_k[i]->value();
    curve->enabled=assist_enabled[i]->isChecked(); curve->update();
  };
  connect(curve_axis,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[redraw](int){redraw();});
  for (int i=0;i<6;++i) {
    connect(assist_enabled[i],&QCheckBox::toggled,this,[redraw](bool){redraw();});
    for (auto* box : {assist_amp[i],assist_width[i],assist_range[i],cart_k[i]})
      connect(box,QOverload<double>::of(&QDoubleSpinBox::valueChanged),this,[redraw](double){redraw();});
  }
  redraw();
  apply->setObjectName("applyTuningButton");
  auto* save = new QPushButton("应用并保存配置");
  save->setObjectName("saveTuningButton");
  form->addRow(apply); form->addRow(save);
  middle->addWidget(tuning);
  auto* monitor = new QGroupBox("实时状态与输出");
  auto* monitor_layout = new QVBoxLayout(monitor);
  values_ = new QLabel("等待设备数据");
  values_->setWordWrap(true); values_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  monitor_layout->addWidget(values_);
  diagnostics_ = new QLabel;
  diagnostics_->setWordWrap(true); diagnostics_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  monitor_layout->addWidget(diagnostics_);
  middle->addWidget(monitor, 1); layout->addLayout(middle);
  auto* log_actions = new QHBoxLayout;
  auto* logs = new QPushButton("打开会话日志");
  auto* note = new QPushButton("记录松手 / 现象备注");
  log_actions->addWidget(logs); log_actions->addWidget(note); layout->addLayout(log_actions);
  event_log_ = new QPlainTextEdit; event_log_->setReadOnly(true);
  event_log_->setMaximumBlockCount(1000); layout->addWidget(event_log_);
  worker_ = new ArmWorker(config);
  worker_->moveToThread(&worker_thread_);
  connect(connect_button, &QPushButton::clicked, this, [this, ip] {
    const auto address = ip->text();
    QMetaObject::invokeMethod(worker_, [this, address] { worker_->connectRobot(address); }, Qt::QueuedConnection);
  });
  connect(disconnect, &QPushButton::clicked, this, [this] {
    if ((snapshot_.low_session || snapshot_.exit_pending) && QMessageBox::warning(this,
        "退出尚未确认", "断开连接不代表停止运动。仅在设备已由现场安全措施控制后，才可断开以恢复通信。确认？",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
    QMetaObject::invokeMethod(worker_, &ArmWorker::disconnectRobot, Qt::QueuedConnection);
  });
  connect(ready, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(this, "确认复位并使能", "此操作会清错并上使能。确认设备状态、支撑和急停条件？",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
      QMetaObject::invokeMethod(worker_, &ArmWorker::setReady, Qt::QueuedConnection);
  });
  connect(disable, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(this, "确认下使能", "确认机械臂有安全支撑，下使能不会掉落？",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
      QMetaObject::invokeMethod(worker_, &ArmWorker::disableServo, Qt::QueuedConnection);
  });
  connect(start, &QPushButton::clicked, this, [this] {
    if (QMessageBox::question(this, "确认进入手柄", "若未使能，将先请求上使能（不自动清错），确认反馈后运动到缓存姿态及配置中心。确认整条路径畅通、人员已离开运动范围、支撑条件合适且实体急停可操作？",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
      QMetaObject::invokeMethod(worker_, &ArmWorker::enableAndStartMitHandle, Qt::QueuedConnection);
  });
  connect(stop, &QPushButton::clicked, this, [this] {
    worker_->requestBatchCancel();
    QMetaObject::invokeMethod(worker_,&ArmWorker::exitHandleMode,Qt::QueuedConnection);
  });
  connect(gravity, &QPushButton::clicked, this, [this] {
    if (QMessageBox::warning(this, "确认无阻尼纯重力测试",
        "将用 1 秒撤除驱动 kp/kd、空间弹簧/阻尼、最低回正力和关节辅助。\n"
        "只发送受保护限制的模型重力；不会保持中心或自动回位，可能漂移、加速或下坠。\n"
        "确认启动保持已结束、现场支撑/防坠措施及实体急停就绪？停止仍会下使能并返回高层。",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) == QMessageBox::Yes)
      QMetaObject::invokeMethod(worker_, &ArmWorker::startGravityTest, Qt::QueuedConnection);
  });
  connect(emergency, &QPushButton::clicked, this, [this] {
    worker_->requestBatchCancel();
    QMetaObject::invokeMethod(worker_,&ArmWorker::emergencyStop,Qt::QueuedConnection);
  });
  auto submit = [this, cart_k, cart_d, joint_k, joint_d, assist_enabled, assist_amp, assist_width, assist_range](bool persist) {
    QJsonArray k, d, kp, kd;
    for (int i = 0; i < 6; ++i) {
      k.append(cart_k[i]->value()); d.append(cart_d[i]->value());
      kp.append(joint_k[i]->value()); kd.append(joint_d[i]->value());
    }
    QJsonArray enabled, amplitude, width, range;
    for (int i=0;i<6;++i) {
      enabled.append(assist_enabled[i]->isChecked()); amplitude.append(assist_amp[i]->value());
      width.append(assist_width[i]->value()); range.append(assist_range[i]->value());
    }
    const QJsonObject assist{{"enabled",enabled},{"amplitude",amplitude},{"inner_transition",width},{"range",range}};
    const QJsonObject tuning{{"near_assist",assist},{"cartesian_stiffness", k}, {"cartesian_damping", d},
                             {"normal_joint_kp", kp}, {"normal_joint_kd", kd}};
    QMetaObject::invokeMethod(worker_, [this,tuning,persist] {
      worker_->applyTuning(tuning, persist);
    }, Qt::QueuedConnection);
  };
  connect(apply, &QPushButton::clicked, this, [submit] { submit(false); });
  connect(save, &QPushButton::clicked, this, [submit] { submit(true); });
  connect(logs, &QPushButton::clicked, this, [this] {
    if (!snapshot_.session_directory.isEmpty()) QDesktopServices::openUrl(QUrl::fromLocalFile(snapshot_.session_directory));
  });
  connect(note, &QPushButton::clicked, this, [this] {
    bool ok=false; const auto text=QInputDialog::getText(this,"现场备注","如：已松手，沿 Z 向下偏移",QLineEdit::Normal,{},&ok);
    if (ok && !text.isEmpty()) QMetaObject::invokeMethod(worker_, [this,text] { worker_->markDiagnostic(text); }, Qt::QueuedConnection);
  });
  connect(worker_, &ArmWorker::message, this, &MainWindow::showMessage);
  connect(worker_, &ArmWorker::snapshotUpdated, this, [=](const ArmSnapshot& s) {
    setProperty("snapshotReceived", true);
    snapshot_=s;
    const bool connected=s.connection==ConnectionState::Connected;
    const bool available=connected && !s.low_session && !s.exit_pending && !s.mit_running;
    connect_button->setEnabled(!connected && !s.low_session && !s.exit_pending);
    disconnect->setEnabled(!s.mit_running && (connected || s.low_session || s.exit_pending));
    ready->setEnabled(available); disable->setEnabled(available);
    const auto entry_block = mitEntryBlockReason(s);
    start->setEnabled(entry_block.isEmpty());
    start->setText(s.servo==ServoState::Enabled ? "进入底层手柄（缓存回接 → 中心）" : "使能并进入底层手柄");
    entry_hint->setText("进入手柄：" + (entry_block.isEmpty() ?
        QString(s.servo==ServoState::Enabled ? "可进入，将经过缓存姿态及配置中心" : "可点击，确认后先使能再进入；不会自动清错") : entry_block));
    start->setToolTip(entry_hint->text());
    stop->setEnabled(s.mit_running || (s.low_session && !s.exit_pending));
    gravity->setEnabled(s.mit_running && !s.mit_startup_holding && !s.gravity_test && !s.exit_pending && s.safety==SafetyState::Normal);
    probe->setEnabled(s.mit_running && s.gravity_test && s.gravity_only_unlimited && !s.exit_pending && s.safety==SafetyState::Normal);
    batch->setEnabled(probe->isEnabled() && !s.friction_batch_active);
    if (s.friction_batch_active) {
      for (auto* button : {disconnect,ready,disable,start,gravity,probe}) button->setEnabled(false);
      stop->setEnabled(true);
    }
    batch_progress->setText(QString("批次：%1，已记录 %2/36 次候选")
        .arg(s.friction_batch_active?"运行中（加力/撤力/等待稳定）":"未运行").arg(s.friction_batch_completed));
    apply->setEnabled(!s.low_session && !s.exit_pending && !s.mit_running && !s.friction_batch_active);
    save->setEnabled(apply->isEnabled());
    tabs->setEnabled(apply->isEnabled());
    banner_->setText(QString("%1 · %2 · %3 · %4")
        .arg(connected ? "已连接" : "未连接")
        .arg(!connected ? "使能未知" : s.servo==ServoState::Enabled ? "已使能" : "未使能")
        .arg(s.safety==SafetyState::Normal ? "正常" : s.safety==SafetyState::Fault ? "故障" : "急停")
        .arg(s.exit_pending ? "退出观察/锁定" : s.mit_running ?
             (s.gravity_test ? (s.gravity_only_unlimited ? "纯重力：零刚度/零阻尼，无自动回中" :
                               "重力试验：过渡/限幅中，暂勿计入测量") : "底层手柄") : "高层监视"));
    values_->setText(connected ? QString("关节 [rad]：%1\n末端 [m, quaternion]：%2\n相对输出：%3\n虚拟力/力矩 [N,Nm]：%4\n高层模式反馈：%5")
        .arg(vectorText(s.joint_position)).arg(vectorText(s.cartesian_pose))
        .arg(vectorText(s.handle_command)).arg(vectorText(s.virtual_wrench)).arg(s.vendor_fsm_state) : "设备未连接，状态未知");
    diagnostics_->setText(QString("日志：%1\nSDK：%2 → %3；%4 ms\nMIT 帧数：%5；最近发送间隔：%6 ms")
        .arg(s.session_directory).arg(s.last_sdk_call).arg(s.last_sdk_result)
        .arg(s.last_sdk_ms,0,'f',2).arg(s.mit_frames).arg(s.last_send_gap_ms,0,'f',2));
    if (!s.error.isEmpty()) showMessage(s.error);
  });
  connect_button->setEnabled(true);
  for (auto* button : {disconnect,ready,disable,start,stop,gravity}) button->setEnabled(false);
  probe->setEnabled(false);
  batch->setEnabled(false);
  worker_thread_.start();
  QMetaObject::invokeMethod(worker_, [this] { worker_->markDiagnostic("最小手柄界面已初始化，未发起硬件操作"); }, Qt::QueuedConnection);
}
MainWindow::~MainWindow() {
  QMetaObject::invokeMethod(worker_, &ArmWorker::disconnectRobot, Qt::BlockingQueuedConnection);
  QMetaObject::invokeMethod(worker_, [this] { delete worker_; worker_=nullptr; }, Qt::BlockingQueuedConnection);
  worker_thread_.quit(); worker_thread_.wait();
}
void MainWindow::closeEvent(QCloseEvent* event) {
  if (snapshot_.mit_running || snapshot_.low_session || snapshot_.exit_pending) {
    QMessageBox::warning(this,"会话尚未结束","请先停止手柄并完成退出；故障锁定时请按现场安全流程处理，不要用关闭程序代替停止。");
    event->ignore(); return;
  }
  QMainWindow::closeEvent(event);
}
void MainWindow::showMessage(const QString& message) {
  if (last_message_ != message) event_log_->appendPlainText(QDateTime::currentDateTime().toString("HH:mm:ss.zzz ") + message);
  last_message_=message;
}
