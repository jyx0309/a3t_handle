#pragma once
#include <QJsonArray>
#include <QJsonObject>
#include "math/handle_controller.hpp"

inline void loadNearAssist(const QJsonObject& h, HandleParameters& p) {
  if (!h.contains("near_assist")) return;
  const auto a = h.value("near_assist").toObject();
  for (int i=0;i<6;++i) {
    p.near_assist_enabled[i]=a.value("enabled").toArray()[i].toBool();
    p.near_assist_amplitude[i]=a.value("amplitude").toArray()[i].toDouble();
    p.near_assist_transition[i]=a.value("inner_transition").toArray()[i].toDouble();
    p.near_assist_range[i]=a.value("range").toArray()[i].toDouble();
  }
}
