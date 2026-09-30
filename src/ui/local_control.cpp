#include "main_window.hpp"
#include "arm/arm_worker.hpp"
#include <QLocalServer>
#include <QLocalSocket>
#include <QJsonDocument>
#include <QJsonArray>
#include <QFileInfo>
#include <QLockFile>
#include <QSet>
#include <QTimer>
#include <QUuid>
#include <memory>

// Opt-in, owner-only Unix socket. Never creates a second hardware connection.
bool MainWindow::enableLocalControl(const QString& path, const QString& ip) {
  auto* server=new QLocalServer(this);
  auto lock=std::make_shared<QLockFile>(path+".lock");
  server->setSocketOptions(QLocalServer::UserAccessOption);
  // Do not unlink another process's socket or silently steal an endpoint.
  if (!QFileInfo(path).isAbsolute() || !lock->tryLock(0) ||
      QFileInfo::exists(path) || QFileInfo(path).isSymLink() || !server->listen(path)) {
    qCritical("Local control socket unavailable (use a fresh absolute path)");
    delete server; return false;
  }
  struct State { QString session=QUuid::createUuid().toString(); QSet<QString> ids; bool pending=false;
    QElapsedTimer clock; qint64 updated=-1; };
  auto state=std::make_shared<State>();
  state->clock.start();
  connect(worker_,&ArmWorker::snapshotUpdated,this,[state](const ArmSnapshot&) { state->updated=state->clock.elapsed(); });
  connect(server,&QLocalServer::newConnection,this,[this,server,state,ip,lock] {
    while (auto* socket=server->nextPendingConnection()) {
      socket->setReadBufferSize(4097);
      connect(socket,&QLocalSocket::disconnected,socket,&QObject::deleteLater);
      QTimer::singleShot(2000,socket,[socket] { socket->disconnectFromServer(); });
      auto buffer=std::make_shared<QByteArray>();
      auto done=std::make_shared<bool>(false);
      connect(socket,&QLocalSocket::readyRead,this,[this,socket,state,ip,buffer,done] {
        if (*done) return;
        buffer->append(socket->readAll());
        if (!buffer->contains('\n') && buffer->size()<=4096) return;
        *done=true;
        auto reply=[socket](QJsonObject obj) {
          socket->write(QJsonDocument(obj).toJson(QJsonDocument::Compact)+'\n');
          socket->disconnectFromServer();
        };
        auto reject=[&](const QString& reason) { reply({{"ok",false},{"error",reason}}); };
        QJsonParseError error;
        const auto doc=QJsonDocument::fromJson(buffer->trimmed(),&error);
        if (buffer->size()>4096 || error.error!=QJsonParseError::NoError || !doc.isObject()) { reject("invalid_request"); return; }
        const auto req=doc.object(); const auto cmd=req.value("command").toString();
        if (cmd=="status") {
          QJsonArray q; for (double x:snapshot_.joint_position) q.append(x);
          reply({{"ok",true},{"session",state->session},{"pending",state->pending},
            {"snapshot_age_ms",state->updated<0?-1:state->clock.elapsed()-state->updated},
            {"connected",snapshot_.connection==ConnectionState::Connected},
            {"enabled",snapshot_.servo==ServoState::Enabled},{"mit",snapshot_.mit_running},
            {"gravity_test",snapshot_.gravity_test},{"gravity_only_unlimited",snapshot_.gravity_only_unlimited},
            {"batch_active",snapshot_.friction_batch_active},{"batch_completed",snapshot_.friction_batch_completed},
            {"exit_pending",snapshot_.exit_pending},{"q",q},{"error",snapshot_.error},
            {"message",last_message_},{"log_directory",snapshot_.session_directory},
            {"warning","Snapshot is cached, not a fresh hardware read. queued is NOT completion."}}); return;
        }
        const QSet<QString> allowed{"connect","start_mit","gravity","probe","batch","stop","emergency"};
        if (!allowed.contains(cmd)) { reject("unsupported_command"); return; }
        if (!req.value("confirm").toBool() || req.value("session").toString()!=state->session) {
          reject("explicit_confirmation_and_current_session_required"); return;
        }
        const QString id=req.value("id").toString();
        if (id.isEmpty() || id.size()>100 || state->ids.contains(id)) { reject("missing_or_duplicate_id"); return; }
        const bool stopping=cmd=="stop" || cmd=="emergency";
        if (!stopping && cmd!="connect" && (state->updated<0 || state->clock.elapsed()-state->updated>1000)) {
          reject("stale_snapshot"); return;
        }
        if (!stopping && (state->pending || state->ids.size()>=4096)) { reject("busy_or_session_request_limit"); return; }
        int joint=req.value("joint").toInt(-1), direction=req.value("direction").toInt(0);
        if (cmd=="probe" && (joint<1 || joint>6 || (direction!=1 && direction!=-1) ||
            req.value("joint").toDouble()!=joint || req.value("direction").toDouble()!=direction)) {
          reject("invalid_joint_or_direction"); return;
        }
        if (cmd=="connect" && snapshot_.connection==ConnectionState::Connected) { reject("already_connected"); return; }
        if (cmd=="start_mit" && (snapshot_.servo!=ServoState::Enabled || snapshot_.safety!=SafetyState::Normal ||
            snapshot_.connection!=ConnectionState::Connected || snapshot_.low_session || snapshot_.exit_pending)) {
          reject("start_requires_connected_enabled_healthy_high_mode"); return;
        }
        state->ids.insert(id);
        if (!stopping) state->pending=true;
        if (stopping) worker_->requestBatchCancel();
        showMessage("本地操作请求："+cmd+" id="+id);
        QMetaObject::invokeMethod(worker_,[this,state,cmd,id,ip,joint,direction,stopping] {
          worker_->markDiagnostic("local_control command="+cmd+" id="+id);
          if (cmd=="connect") worker_->connectRobot(ip);
          else if (cmd=="start_mit") worker_->startMitHandle();
          else if (cmd=="gravity") worker_->startGravityTest();
          else if (cmd=="probe") worker_->startFrictionProbe(joint-1,direction);
          else if (cmd=="batch") worker_->startFrictionBatch();
          else if (cmd=="stop") worker_->exitHandleMode();
          else if (cmd=="emergency") worker_->emergencyStop();
          if (!stopping) QMetaObject::invokeMethod(this,[state] { state->pending=false; },Qt::QueuedConnection);
        },Qt::QueuedConnection);
        reply({{"ok",true},{"queued",true},{"id",id}});
      });
    }
  });
  showMessage("本地控制入口已启用："+path+"；未自动连接或启动机械臂。");
  return true;
}
