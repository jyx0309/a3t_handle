#pragma once
#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <cmath>

inline QString validateConfig(const QJsonObject& config) {
  if (config.contains("gravity")) {
    if (!config.value("gravity").isObject()) return "gravity must be an object";
    const auto gravity=config.value("gravity").toObject();
    const auto source=gravity.value("source").toString();
    if (source!="sdk" && source!="urdf") return "gravity.source must be sdk or urdf";
    if (source=="urdf" && gravity.value("urdf_path").toString().isEmpty()) return "gravity.urdf_path required";
    if (gravity.contains("compensation")) {
      if (!gravity.value("compensation").isObject()) return "gravity.compensation must be an object";
      const auto compensation=gravity.value("compensation").toObject();
      if (!compensation.value("enabled").isBool()) return "gravity.compensation.enabled must be boolean";
      if (compensation.value("enabled").toBool()) {
        if (source!="urdf") return "gravity compensation requires gravity.source=urdf";
        if (compensation.value("base_sha256").toString().isEmpty()) return "gravity compensation base_sha256 required";
        const auto parameters=compensation.value("parameter_delta").toArray();
        const auto residual=compensation.value("residual_nm").toArray();
        if (parameters.size()!=13 || residual.size()!=6) return "gravity compensation parameter count invalid";
        for (const auto value : parameters)
          if (!value.isDouble() || !std::isfinite(value.toDouble())) return "Invalid gravity compensation parameter";
        for (const auto value : residual)
          if (!value.isDouble() || !std::isfinite(value.toDouble()) || std::abs(value.toDouble())>0.5)
            return "Invalid gravity compensation residual";
      }
    }
  }
  for (const auto* section : {"mit", "handle"})
    if (!config.value(section).isObject()) return QString("Missing object: %1").arg(section);
  const auto mit = config.value("mit").toObject();
  for (const auto* flag : {"allow_real_mit"})
    if (!mit.value(flag).isBool()) return QString("mit.%1 must be boolean").arg(flag);
  for (const auto* name : {"period_ms", "watchdog_ms", "max_failures"}) {
    const auto v = mit.value(name);
    const double n = v.toDouble(-1);
    if (!v.isDouble() || !std::isfinite(n) || n < 1 || n > 1000000 || std::floor(n) != n)
      return QString("mit.%1 must be a positive integer <= 1000000").arg(name);
  }
  if (mit.value("watchdog_ms").toInt() < mit.value("period_ms").toInt()) return "watchdog_ms < period_ms";
  for (const auto* name : {"joint_kp", "joint_kd"}) {
    const auto v = mit.value(name);
    if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble() < 0)
      return QString("Invalid mit.%1").arg(name);
  }
  const auto recovery_speed = mit.value("cache_recovery_speed_level");
  // Optional paired, explicit normal-mode gains. Device-reported gains may
  // contain the last gravity-test command, not factory defaults.
  if (mit.contains("normal_joint_kp") || mit.contains("normal_joint_kd")) {
    for (const auto* name : {"normal_joint_kp", "normal_joint_kd"}) {
      const auto values = mit.value(name).toArray();
      if (values.size() != 6) return QString("mit.%1 must contain six normal-mode gains").arg(name);
      for (const auto value : values)
        if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() < 0.0)
          return QString("mit.%1 must contain non-negative finite normal-mode gains").arg(name);
    }
  }
  if (!recovery_speed.isDouble() || !std::isfinite(recovery_speed.toDouble()) ||
      recovery_speed.toDouble() < 0.0 || recovery_speed.toDouble() > 10.0)
    return "Invalid mit.cache_recovery_speed_level";
  const auto center = mit.value("center_joint_position_rad").toArray();
  if (center.size() != 6) return "mit.center_joint_position_rad must contain six joint angles";
  for (const auto value : center)
    if (!value.isDouble() || !std::isfinite(value.toDouble())) return "Invalid mit.center_joint_position_rad";
  for (const auto* name : {"joint_center_assist_kp_nm_rad", "joint_center_assist_kd_nm_s_rad"}) {
    const auto values = mit.value(name).toArray();
    if (values.size() != center.size()) return QString("mit.%1 must match center DOF").arg(name);
    for (const auto value : values)
      if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() < 0.0)
        return QString("Invalid mit.%1").arg(name);
  }
  const auto assist_limit = mit.value("joint_center_assist_limit_nm");
  if (!assist_limit.isDouble() || !std::isfinite(assist_limit.toDouble()) || assist_limit.toDouble() <= 0.0)
    return "Invalid mit.joint_center_assist_limit_nm";
  const auto assist_activation = mit.value("joint_center_assist_activation_rad");
  if (!assist_activation.isDouble() || !std::isfinite(assist_activation.toDouble()) ||
      assist_activation.toDouble() <= 0.0 || assist_activation.toDouble() > 1.0)
    return "Invalid mit.joint_center_assist_activation_rad";
  for (const auto* name : {"static_friction_positive_nm", "static_friction_negative_nm"}) {
    const auto values = mit.value(name).toArray();
    if (values.size() != center.size()) return QString("mit.%1 must match center DOF").arg(name);
    for (const auto value : values)
      if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() < 0.0 || value.toDouble() > 2.0)
        return QString("Invalid mit.%1").arg(name);
  }
  for (const auto* name : {"static_friction_activation_nm", "static_friction_velocity_rad_s"}) {
    const auto value = mit.value(name);
    if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() <= 0.0)
      return QString("Invalid mit.%1").arg(name);
  }
  const auto handle = config.value("handle").toObject();
  if (handle.contains("near_assist")) {
    if (!handle.value("near_assist").isObject()) return "Invalid near_assist object";
    const auto a=handle.value("near_assist").toObject();
    for (const auto* key : {"enabled", "amplitude", "inner_transition", "range"})
      if (!a.value(key).isArray() || a.value(key).toArray().size()!=6)
        return QString("near_assist.%1 requires six entries").arg(key);
    for (int i=0;i<6;++i) {
      if (!a.value("enabled").toArray()[i].isBool()) return "Invalid near_assist.enabled";
      for (const auto* key : {"amplitude", "inner_transition", "range"}) {
        const auto v=a.value(key).toArray()[i];
        if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble()<0)
          return QString("Invalid near_assist.%1").arg(key);
      }
      const double d=handle.value(i<3?"position_deadband_m":"rotation_deadband_rad").toDouble();
      const double w=a.value("inner_transition").toArray()[i].toDouble();
      const double r=a.value("range").toArray()[i].toDouble();
      if (w<=0 || r<=d || w>=0.75*(r-d))
        return QString("near_assist axis %1: require 0 < transition < 0.75*(range-deadband)").arg(i+1);
    }
  }
  if (handle.contains("cartesian_stiffness") || handle.contains("cartesian_damping")) {
    for (const auto* key : {"cartesian_stiffness", "cartesian_damping"}) {
      const auto values = handle.value(key).toArray();
      if (values.size() != 6) return QString("handle.%1 requires six values").arg(key);
      for (const auto value : values)
        if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() < 0)
          return QString("Invalid handle.%1").arg(key);
    }
  }
  for (const auto* name : {"position_scale", "rotation_scale", "position_deadband_m", "position_return_breakaway_force_n", "position_return_transition_m", "position_stiffness", "position_damping", "rotation_stiffness", "rotation_damping"}) {
    const auto v = handle.value(name);
    if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble() < 0) return QString("Invalid handle.%1").arg(name);
  }
  if (handle.value("position_return_transition_m").toDouble() <= 0.0) return "position_return_transition_m must be positive";
  for (const auto* name : {"rotation_deadband_rad", "rotation_return_breakaway_torque_nm", "rotation_return_transition_rad"}) {
    const auto v = handle.value(name);
    if (!v.isDouble() || !std::isfinite(v.toDouble()) || v.toDouble() < 0.0)
      return QString("Invalid handle.%1").arg(name);
  }
  if (handle.value("rotation_return_transition_rad").toDouble() <= 0.0)
    return "rotation_return_transition_rad must be positive";
  for (const auto* name : {"position_axis_scale", "rotation_axis_scale"}) {
    const auto values = handle.value(name).toArray();
    if (values.size() != 3) return QString("handle.%1 must contain three axis scales").arg(name);
    for (const auto value : values)
      if (!value.isDouble() || !std::isfinite(value.toDouble()) || value.toDouble() <= 0.0 || value.toDouble() > 1.0)
        return QString("Invalid handle.%1").arg(name);
  }
  if (handle.value("position_scale").toDouble() <= 0 || handle.value("rotation_scale").toDouble() <= 0) return "Scales must be positive";
  const auto poll = config.value("poll_interval_ms");
  if (!poll.isDouble() || poll.toDouble() < 1 || poll.toDouble() > 1000 || std::floor(poll.toDouble()) != poll.toDouble()) return "Invalid poll_interval_ms";
  return {};
}
