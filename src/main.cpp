#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDir>
#include <QDebug>
#include <QJsonParseError>
#include <QTimer>
#include <QTranslator>
#include <QLibraryInfo>
#include <QFont>
#include <QPushButton>
#include <QDoubleSpinBox>
#include "config/config_validation.hpp"

#include "ui/main_window.hpp"

int main(int argc, char* argv[]) {
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  QApplication app(argc, argv);
  app.setFont(QFont("Noto Sans CJK SC", 11));
  QTranslator qt_chinese;
  qt_chinese.load("qtbase_zh_CN", QLibraryInfo::location(QLibraryInfo::TranslationsPath));
  app.installTranslator(&qt_chinese);
  QJsonObject config;
  // Resolve from executable, not the shell's current directory. Explicit override
  // is useful for installed builds and isolated offline tests.
  const bool smoke_test = argc > 1 && QString::fromLocal8Bit(argv[1]) == "--smoke-test";
  const QString path = argc > 1 && !smoke_test ? QString::fromLocal8Bit(argv[1]) :
      QDir(QCoreApplication::applicationDirPath()).filePath("../config/handle.json");
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) { qCritical() << "Cannot open configuration:" << path; return 2; }
  QJsonParseError parse;
  const auto doc = QJsonDocument::fromJson(file.readAll(), &parse);
  if (parse.error != QJsonParseError::NoError || !doc.isObject()) {
    qCritical() << "Invalid configuration:" << path << parse.errorString(); return 2;
  }
  config = doc.object();
  config["configuration_file"] = QFileInfo(file).absoluteFilePath();
  const auto error = validateConfig(config);
  if (!error.isEmpty()) { qCritical() << path << error; return 2; }
  qInfo() << "Configuration:" << QFileInfo(file).absoluteFilePath();
  MainWindow window(config);
  const QString control_path=qEnvironmentVariable("A3T_CONTROL_SOCKET");
  if (!smoke_test && !control_path.isEmpty() &&
      !window.enableLocalControl(control_path,config.value("controller_ip").toString())) return 4;
  window.resize(1280, 860);
  window.show();
  if (smoke_test) QTimer::singleShot(300, &app, [&] {
    for (const auto* axis : {"X", "Y", "Z", "Rx", "Ry", "Rz"}) {
      if (!window.findChild<QDoubleSpinBox*>(QString("cartesianK") + axis) ||
          !window.findChild<QDoubleSpinBox*>(QString("cartesianD") + axis)) {
        app.exit(3); return;
      }
    }
    for (int i = 1; i <= 6; ++i)
      if (!window.findChild<QDoubleSpinBox*>(QString("mitKp%1").arg(i)) ||
          !window.findChild<QDoubleSpinBox*>(QString("mitKd%1").arg(i))) {
        app.exit(3); return;
      }
    const auto* gravity = window.findChild<QPushButton*>("gravityTestButton");
    if (!gravity || gravity->isEnabled()) {
      qCritical() << "Smoke test failed: gravity test entry missing or enabled while disconnected";
      app.exit(3);
      return;
    }
    if (!window.property("snapshotReceived").toBool()) {
      qCritical() << "Smoke test failed: no queued worker snapshot received";
      app.exit(3);
      return;
    }
    const QString screenshots = qEnvironmentVariable("A3T_UI_SCREENSHOT_DIR");
    if (!screenshots.isEmpty() && QDir(screenshots).exists()) {
      window.grab().save(QDir(screenshots).filePath("main.png"));
    }
    app.quit();
  });
  return app.exec();
}
