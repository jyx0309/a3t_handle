#pragma once
#include <QPainter>
#include <QPainterPath>
#include <QWidget>
#include "math/handle_controller.hpp"

class AssistCurve : public QWidget {
 public:
  explicit AssistCurve(QWidget* parent=nullptr) : QWidget(parent) { setMinimumSize(360,160); }
  double deadband{0.0005}, width{0.0005}, range{0.01}, amplitude{0.25}, stiffness{30};
  bool enabled{false};
 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this); p.fillRect(rect(), Qt::white);
    p.setRenderHint(QPainter::Antialiasing);
    const QRectF area(50,25,QWidget::width()-65,height()-55);
    if (range<=deadband || width<=0 || width>=0.75*(range-deadband)) {
      p.drawText(rect(), Qt::AlignCenter, "范围需大于死区，且内过渡需小于可用范围的 75%"); return;
    }
    const double xmax=range*1.25;
    const double ymax=std::max(1e-6,stiffness*xmax+amplitude);
    p.setPen(Qt::gray); p.drawLine(area.bottomLeft(),area.bottomRight()); p.drawLine(area.bottomLeft(),area.topLeft());
    auto draw=[&](bool total, QColor color) {
      QPainterPath path;
      for (int i=0;i<=240;++i) {
        const double x=xmax*i/240;
        const double a=enabled?HandleController::nearAssist(x,deadband,width,range,amplitude):0;
        const double y=a+(total?stiffness*std::max(0.0,x-deadband):0);
        QPointF point(area.left()+area.width()*x/xmax,area.bottom()-area.height()*y/ymax);
        if (!i) path.moveTo(point); else path.lineTo(point);
      }
      p.setPen(QPen(color,2)); p.drawPath(path);
    };
    draw(true,QColor("#2563eb")); draw(false,QColor("#ea580c"));
    p.setPen(Qt::black);
    p.drawText(5,16,"蓝：弹簧＋新辅助  橙：新辅助（静态幅值）");
    p.drawText(3,40,QString::number(ymax,'g',3));
    p.drawText(50,height()-5,QString("偏差绝对值 → %1（m 或 rad）；未含其他补偿").arg(xmax));
  }
};
