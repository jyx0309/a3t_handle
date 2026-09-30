#ifdef NDEBUG
#undef NDEBUG
#endif
#include "logging/session_logger.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QJsonDocument>
#include <cassert>
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  QCoreApplication app(argc, argv);
  QTemporaryDir root;
  assert(root.isValid());
  QString directory;
  {
    SessionLogger logger(root.path());
    assert(logger.isReady());
    directory = logger.sessionDirectory();
    logger.command("test", "quoted,\"text\"");
    logger.state(ArmSnapshot{});
    HandleOutput output;
    output.valid = true;
    logger.handle(output);
    for (int i = 0; i < 2000; ++i) {
      logger.diagnostic({{"type", "test"}, {"sequence", i}});
      logger.performance("test", i * 0.001);
    }
    assert(logger.isReady());
    // The writer must flush while the producer is idle, not only at shutdown.
    bool visible = false;
    for (int i = 0; i < 200 && !visible; ++i) {
      QFile file(directory + "/diagnostics.jsonl");
      assert(file.open(QIODevice::ReadOnly));
      visible = file.readAll().contains("\"sequence\":1999");
      if (!visible) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    assert(visible);
    // Immediate destruction must drain remaining queued records in order.
    for (int i = 2000; i < 2100; ++i)
      logger.diagnostic({{"type", "test"}, {"sequence", i}});
  }
  QFile json(directory + "/diagnostics.jsonl");
  assert(json.open(QIODevice::ReadOnly));
  int count = 0;
  QString last_action;
  double previous = -1;
  while (!json.atEnd()) {
    QJsonParseError error;
    const auto doc = QJsonDocument::fromJson(json.readLine(), &error);
    assert(error.error == QJsonParseError::NoError);
    const auto record = doc.object();
    assert(record["monotonic_ms"].toDouble() >= previous);
    previous = record["monotonic_ms"].toDouble();
    if (record["type"] == "test") assert(record["sequence"].toInt() == count++);
    if (record["type"] == "event") last_action = record["action"].toString();
  }
  assert(count == 2100 && last_action == "session_finished");
  QFile perf(directory + "/performance.csv");
  assert(perf.open(QIODevice::ReadOnly));
  assert(perf.readAll().count('\n') == 2001);
  QFile obstruction(root.path() + "/not_a_directory");
  assert(obstruction.open(QIODevice::WriteOnly));
  SessionLogger failed(obstruction.fileName());
  assert(!failed.isReady());
}
