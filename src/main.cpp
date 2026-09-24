#include <QApplication>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>

#include "ui/main_window.hpp"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QJsonObject config;
  QFile file("config/handle.json");
  if (file.open(QIODevice::ReadOnly)) config = QJsonDocument::fromJson(file.readAll()).object();
  MainWindow window(config);
  window.resize(920, 280);
  window.show();
  return app.exec();
}
