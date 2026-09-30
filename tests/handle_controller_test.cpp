#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "control/mit_handle_controller.hpp"
#include "control/pv_trajectory.hpp"
#include "arm/mit_entry.hpp"
#include "control/mit_startup_hold.hpp"
#include "control/friction_probe.hpp"
#include "control/friction_batch.hpp"
#include "math/handle_controller.hpp"
#include "math/urdf_gravity.hpp"
#include <QFileInfo>
#include <QDir>
#include "safety/safety_monitor.hpp"
#include "config/config_validation.hpp"
#include <QFile>
#include <QJsonDocument>

namespace {
constexpr double kEpsilon = 1e-9;

bool near(double actual, double expected, double tolerance = kEpsilon) {
  return std::abs(actual - expected) <= tolerance;
}

std::array<double, 7> identityPose() { return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}; }

void testHandleDeadbandAndWrench() {
  HandleController controller;
  HandleParameters parameters;
  parameters.position_deadband_m = 0.01;
  parameters.position_stiffness = 10.0;
  parameters.position_damping = 0.0;
  parameters.filter_alpha = 1.0;
  controller.setParameters(parameters);
  controller.captureCenter(identityPose(), 0.0);

  auto pose = identityPose();
  pose[0] = 0.005;
  const auto inside_deadband = controller.update(pose, 0.1);
  assert(inside_deadband.valid);
  assert(near(inside_deadband.filtered_error[0], 0.0));
  assert(near(inside_deadband.virtual_wrench[0], 0.0));

  pose[0] = 0.11;
  const auto outside_deadband = controller.update(pose, 0.2);
  assert(near(outside_deadband.filtered_error[0], 0.10));
  assert(near(outside_deadband.virtual_wrench[0], -1.0));
}

void testSmoothCartesianBreakawayForce() {
  HandleController controller;
  HandleParameters parameters;
  parameters.position_deadband_m = 0.001;
  parameters.position_return_breakaway_force_n = 0.6;
  parameters.position_return_transition_m = 0.002;
  parameters.position_stiffness = 10.0;
  parameters.position_damping = 0.0;
  parameters.filter_alpha = 1.0;
  controller.setParameters(parameters);
  controller.captureCenter(identityPose(), 0.0);
  auto pose = identityPose();
  pose[0] = 0.002;  // Half way through the 2 mm smooth breakaway transition.
  pose[1] = 0.002;
  const auto output = controller.update(pose, 0.1);
  // Same Cartesian displacement produces the same Cartesian centering force
  // on X and Y.  10 * 1 mm spring force + 0.6 * smoothstep(0.5).
  assert(near(output.virtual_wrench[0], -0.31));
  assert(near(output.virtual_wrench[1], -0.31));
}

void testCartesianAxisScaling() {
  HandleController controller;
  HandleParameters parameters;
  parameters.position_deadband_m = 0.0;
  parameters.position_stiffness = 10.0;
  parameters.position_damping = 0.0;
  parameters.position_axis_scale = {0.5, 1.0, 0.25};
  parameters.filter_alpha = 1.0;
  controller.setParameters(parameters);
  controller.captureCenter(identityPose(), 0.0);
  auto pose = identityPose();
  pose[0] = pose[1] = pose[2] = 0.1;
  const auto output = controller.update(pose, 0.1);
  assert(near(output.virtual_wrench[0], -0.5));
  assert(near(output.virtual_wrench[1], -1.0));
  assert(near(output.virtual_wrench[2], -0.25));
}

void testShortestQuaternionRotation() {
  HandleController controller;
  HandleParameters parameters;
  parameters.rotation_deadband_rad = 0.0;
  parameters.rotation_stiffness = 1.0;
  parameters.rotation_damping = 0.0;
  parameters.filter_alpha = 1.0;
  controller.setParameters(parameters);
  controller.captureCenter(identityPose(), 0.0);
  auto pose = identityPose();
  const double angle = 0.2;
  pose[5] = std::sin(angle / 2.0);
  pose[6] = std::cos(angle / 2.0);
  const auto output = controller.update(pose, 0.1);
  assert(near(output.raw_error[5], angle));
  assert(near(output.filtered_error[5], angle));
  assert(near(output.virtual_wrench[5], -output.filtered_error[5]));
}

void testMitMappingAndLimits() {
  MitHandleController controller;
  controller.setJacobianLayout(MitHandleController::JacobianLayout::JointRows);
  MitSafetyParameters parameters;
  parameters.torque_limit_nm = 3.0;
  parameters.torque_rate_limit_nm_s = 10.0;
  parameters.control_period_s = 0.1;
  controller.setSafetyParameters(parameters);
  controller.setJointGains({2.0, 3.0}, {0.2, 0.3});
  HandleOutput handle;
  handle.valid = true;
  handle.dt = 0.1;
  handle.virtual_wrench[0] = 2.0;
  const std::vector<double> jacobian = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
  };
  const auto first = controller.compute(handle, {0.0, 0.0}, {0.0, 0.0}, {0.5, -0.5}, jacobian, 2, 6);
  assert(first.valid);
  assert(near(first.torque[0], 0.5));
  assert(near(first.torque[1], -0.5));
  assert(near(first.velocity[0], 0.0));
  assert(near(first.kp[0], 2.0));
  assert(near(first.kp[1], 3.0));
  assert(near(first.kd[0], 0.2));
  assert(near(first.kd[1], 0.3));

  const auto ramped = controller.compute(handle, {0.0, 0.0}, {1.0, -1.0}, {0.5, -0.5}, jacobian, 2, 6);
  assert(ramped.valid);
  assert(near(ramped.torque[0], 2.5));
  assert(near(ramped.torque[1], 0.5));
  assert(near(ramped.velocity[0], 0.0));

  handle.virtual_wrench[0] = 20.0;
  const auto limited = controller.compute(handle, {0.0, 0.0}, {0.0, 0.0}, {0.5, -0.5}, jacobian, 2, 6);
  assert(limited.valid);
  assert(near(limited.torque[0], 20.5));

  handle.virtual_wrench[0] = std::numeric_limits<double>::quiet_NaN();
  const auto invalid = controller.compute(handle, {0.0, 0.0}, {0.0, 0.0}, {0.5, -0.5}, jacobian, 2, 6);
  assert(!invalid.valid);
}

void testMitPerJointTorqueLimit() {
  MitHandleController controller;
  controller.setJacobianLayout(MitHandleController::JacobianLayout::JointRows);
  MitSafetyParameters parameters;
  parameters.torque_limit_nm = 3.0;
  parameters.joint_torque_limit_nm = {3.0, 4.0};
  parameters.torque_rate_limit_nm_s = 1000.0;
  parameters.control_period_s = 0.1;
  controller.setSafetyParameters(parameters);
  HandleOutput handle;
  handle.valid = true;
  handle.dt = 0.1;
  handle.virtual_wrench[0] = 8.0;
  const std::vector<double> jacobian = {
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
  };
  controller.compute(handle, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, jacobian, 2, 6);
  const auto result = controller.compute(handle, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, jacobian, 2, 6);
  assert(result.valid);
  assert(near(result.torque[0], 8.0));
  assert(near(result.torque[1], 4.0));
  assert(!result.torque_saturated[0]);
  assert(!result.torque_saturated[1]);
}

void testBaseFrameRotation() {
  HandleController controller;
  auto center = identityPose();
  const double s = std::sqrt(0.5);
  center[5] = s; center[6] = s;  // center is rotated 90 degrees about base Z
  controller.captureCenter(center, 0.0);
  auto pose = center;
  // Apply a 0.2 rad rotation about BASE X: q_x * q_center.
  pose[3] = std::sin(0.1) * s;
  pose[4] = -std::sin(0.1) * s;
  pose[5] = std::cos(0.1) * s;
  pose[6] = std::cos(0.1) * s;
  auto result = controller.update(pose, 0.01);
  assert(near(result.raw_error[3], 0.2));
  assert(near(result.raw_error[4], 0.0));
  assert(near(result.raw_error[5], 0.0));
  for (int axis = 3; axis < 7; ++axis) pose[axis] *= -1.0;
  result = controller.update(pose, 0.02);
  assert(near(result.raw_error[3], 0.2));
}

void testSquareJacobianLayout() {
  MitSafetyParameters safety;
  safety.torque_limit_nm = 100.0;
  safety.torque_rate_limit_nm_s = 10000.0;
  HandleOutput handle;
  handle.valid = true;
  handle.dt = 0.01;
  handle.virtual_wrench[0] = 2.0;
  std::vector<double> j(36, 0.0), zero(6, 0.0);
  j[1] = 3.0; j[6] = 7.0;  // nonsymmetric: distinguishes J from J transpose
  MitHandleController controller;
  controller.setSafetyParameters(safety);
  controller.compute(handle, zero, zero, zero, j, 6, 6);
  auto result = controller.compute(handle, zero, zero, zero, j, 6, 6);
  assert(result.valid && near(result.torque[1], 6.0));
  controller.setJacobianLayout(MitHandleController::JacobianLayout::JointRows);
  controller.reset();
  controller.compute(handle, zero, zero, zero, j, 6, 6);
  result = controller.compute(handle, zero, zero, zero, j, 6, 6);
  assert(result.valid && near(result.torque[1], 14.0));
}

void testMitMotionSafety() {
  SafetyMonitor monitor;
  MotionSafetyParameters parameters;
  parameters.require_deadman_for_mit = true;
  parameters.require_workspace_for_mit = true;
  parameters.workspace_configured = true;
  parameters.workspace_min_m = {-1.0, -1.0, -1.0};
  parameters.workspace_max_m = {1.0, 1.0, 1.0};
  parameters.joint_limit_margin_rad = 0.1;
  parameters.joint_velocity_fraction = 0.5;
  monitor.setMotionSafetyParameters(parameters);
  monitor.setVendorJointLimits({-2.0, -2.0}, {2.0, 2.0}, {4.0, 4.0});
  QString reason;
  assert(monitor.isSafeForMit({0.0, 0.0}, {0.0, 0.0}, identityPose(), false, &reason));
  assert(monitor.isSafeForMit({0.0, 0.0}, {1.0, 1.0}, identityPose(), true, &reason));
  assert(monitor.isSafeForMit({1.95, 0.0}, {0.0, 0.0}, identityPose(), true, &reason));
  assert(monitor.isSafeForMit({0.0, 0.0}, {2.1, 0.0}, identityPose(), true, &reason));
  auto outside_workspace = identityPose();
  outside_workspace[0] = 1.1;
  assert(monitor.isSafeForMit({0.0, 0.0}, {0.0, 0.0}, outside_workspace, true, &reason));
  parameters.enforce_joint_velocity_limit = false;
  monitor.setMotionSafetyParameters(parameters);
  assert(monitor.isSafeForMit({0.0, 0.0}, {5.0, -5.0}, identityPose(), true, &reason));
  assert(!monitor.isSafeForMit({0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}, identityPose(), true, &reason));
  assert(monitor.isSafeForMit({1.95, 0.0}, {5.0, 0.0}, identityPose(), true, &reason));
  assert(monitor.isSafeForMit({0.0, 0.0}, {5.0, 0.0}, outside_workspace, true, &reason));
  assert(monitor.isSafeForMit({0.0, 0.0}, {5.0, 0.0}, identityPose(), false, &reason));
  parameters.enforce_joint_velocity_limit = true;
  monitor.setMotionSafetyParameters(parameters);
  assert(monitor.isSafeForMit({0.0, 0.0}, {5.0, 0.0}, identityPose(), true, &reason));
}
}  // namespace

void testInvalidPose() {
  HandleController controller;
  controller.captureCenter({}, 0);
  assert(!controller.hasCenter());
  controller.captureCenter(identityPose(), 0);
  assert(!controller.update({}, 0.01).valid);
  auto bad = identityPose();
  bad[0] = std::numeric_limits<double>::quiet_NaN();
  assert(!controller.update(bad, 0.01).valid);
  controller.clearCenter();
  assert(!controller.update(identityPose(), 0.02).valid);
}


void testConfiguration() {
  QFile file(TEST_CONFIG_PATH);
  assert(file.open(QIODevice::ReadOnly));
  auto config = QJsonDocument::fromJson(file.readAll()).object();
  assert(validateConfig(config).isEmpty());
  {
    auto candidate=config; auto h=candidate.value("handle").toObject();
    QJsonObject a{{"enabled",QJsonArray{true,false,false,false,false,false}},
      {"amplitude",QJsonArray{0.25,0,0,0.02,0,0}},
      {"inner_transition",QJsonArray{0.0005,0.0005,0.0005,0.005,0.005,0.005}},
      {"range",QJsonArray{0.01,0.01,0.01,0.1,0.1,0.1}}};
    h["near_assist"]=a; candidate["handle"]=h;
    assert(validateConfig(candidate).isEmpty());
    assert(validateConfig(QJsonDocument::fromJson(QJsonDocument(candidate).toJson()).object()).isEmpty());
    a["range"]=QJsonArray{0,0,0,0,0,0}; h["near_assist"]=a; candidate["handle"]=h;
    assert(!validateConfig(candidate).isEmpty());
    a["enabled"]=QJsonArray{true}; h["near_assist"]=a; candidate["handle"]=h;
    assert(!validateConfig(candidate).isEmpty());
  }
  {
    auto candidate = config;
    auto h = candidate.value("handle").toObject();
    h["cartesian_stiffness"] = QJsonArray{0,1,2,3,4,5};
    h["cartesian_damping"] = QJsonArray{5,4,3,2,1,0};
    candidate["handle"] = h;
    assert(validateConfig(candidate).isEmpty());
    h["cartesian_damping"] = QJsonArray{1,2};
    candidate["handle"] = h;
    assert(!validateConfig(candidate).isEmpty());
    h["cartesian_damping"] = QJsonArray{0,0,0,0,0,-1};
    candidate["handle"] = h;
    assert(!validateConfig(candidate).isEmpty());
    h.remove("cartesian_damping");
    candidate["handle"] = h;
    assert(!validateConfig(candidate).isEmpty());
    h.remove("cartesian_stiffness");
    candidate["handle"] = h;
    assert(validateConfig(candidate).isEmpty()); // Legacy configuration.
  }
  {
    auto invalid = config;
    auto mit = invalid.value("mit").toObject();
    mit["normal_joint_kp"] = QJsonArray{80.0, 80.0, 50.0, 50.0, 50.0, 50.0};
    mit["normal_joint_kd"] = QJsonArray{5.0, 5.0, 1.0, 1.0, 1.0, 1.0};
    invalid["mit"] = mit;
    assert(validateConfig(invalid).isEmpty());
    mit.remove("normal_joint_kd"); invalid["mit"] = mit;
    assert(!validateConfig(invalid).isEmpty());
    mit["normal_joint_kd"] = QJsonArray{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; invalid["mit"] = mit;
    assert(validateConfig(invalid).isEmpty());
    mit["normal_joint_kd"] = QJsonArray{-1.0, 0.0, 0.0, 0.0, 0.0, 0.0}; invalid["mit"] = mit;
    assert(!validateConfig(invalid).isEmpty());
  }
  auto mit = config.value("mit").toObject();
  mit["period_ms"] = 0.5;
  config["mit"] = mit;
  assert(!validateConfig(config).isEmpty());
  assert(!validateConfig({}).isEmpty());
}

void testReturnForceAndAssistRegression() {
  HandleController h;
  HandleParameters p;
  p.position_stiffness = p.position_damping = 0;
  p.position_axis_scale = {0.35, 0.35, 1};
  p.position_return_breakaway_force_n = 1;
  p.rotation_stiffness = p.rotation_damping = 0;
  p.rotation_return_breakaway_torque_nm = 0.08;
  p.filter_alpha = 1;
  h.setParameters(p);
  h.captureCenter(identityPose(), 0);
  auto pose = identityPose();
  pose[0] = 0.06; pose[1] = -0.06; pose[2] = 0.06;
  pose[5] = std::sin(0.1); pose[6] = std::cos(0.1);
  auto out = h.update(pose, 0.01);
  assert(near(out.virtual_wrench[0], -1));
  assert(near(out.virtual_wrench[1], 1));
  assert(near(out.virtual_wrench[2], -1));
  assert(near(out.virtual_wrench[5], -0.08));
  out = h.update(identityPose(), 0.02);
  for (double f : out.virtual_wrench) assert(near(f, 0));

  MitHandleController m;
  MitSafetyParameters s;
  s.torque_limit_nm = 3; s.torque_rate_limit_nm_s = 1000;
  m.setSafetyParameters(s);
  m.setJointCenterAssist({0, 0}, {3, 3}, {0, 0}, 0.8, 0.3);
  m.setStaticFrictionCompensation({0.2, 0.3}, {0.4, 0.5}, 0.1, 0.03);
  m.setJacobianLayout(MitHandleController::JacobianLayout::JointRows);
  HandleOutput input; input.valid = true; input.dt = 0.01;
  std::vector<double> j(12, 0);
  m.compute(input, {0.5, -0.05}, {0, 0}, {0, 0}, j, 2, 6);
  auto c = m.compute(input, {0.5, -0.05}, {0, 0}, {0, 0}, j, 2, 6);
  assert(c.valid && c.joint_assist_torque[0] < 0 && c.joint_assist_torque[1] > 0);
  assert(std::abs(c.joint_assist_torque[0]) <= 0.8);
  assert(near(c.static_friction_torque[0], -0.4));
  assert(near(c.static_friction_torque[1], 0.3));
  auto moving = m.compute(input, {0.5, -0.05}, {0.03, 0.03}, {0, 0}, j, 2, 6);
  assert(near(moving.static_friction_torque[0], 0.0));
  assert(near(moving.static_friction_torque[1], 0.0));
  auto farther = m.compute(input, {1.0, -0.05}, {0, 0}, {0, 0}, j, 2, 6);
  assert(near(c.joint_assist_torque[1], farther.joint_assist_torque[1]));
}

void testGravityOnlyTransition() {
  MitHandleController controller;
  MitSafetyParameters limits;
  limits.torque_limit_nm = 10.0;
  limits.torque_rate_limit_nm_s = 100.0;
  controller.setSafetyParameters(limits);
  controller.setJointGains({50.0}, {5.0});
  controller.setJointCenterAssist({0.0}, {3.0}, {0.5}, 0.8, 0.3);
  HandleOutput handle;
  handle.valid = true; handle.dt = 0.01;
  handle.virtual_wrench = {2.0, 0, 0, 0, 0, 0};
  const std::vector<double> jacobian{1, 0, 0, 0, 0, 0};
  auto compute = [&](double scale, double gravity = -3.0) {
    return controller.compute(handle, {0.1}, {0.2}, {gravity}, jacobian, 6, 1, scale);
  };
  assert(compute(1.0).valid);
  const auto normal = compute(1.0);
  const auto half = compute(0.5);
  assert(near(half.kp[0], 25.0) && near(half.kd[0], 2.5));
  assert(near(half.cartesian_torque[0], normal.cartesian_torque[0] * 0.5));
  assert(near(half.joint_assist_torque[0], normal.joint_assist_torque[0] * 0.5));
  MitCommand pure;
  for (int i = 0; i < 10; ++i) pure = compute(0.0);
  assert(pure.valid && near(pure.kp[0], 0.0) && near(pure.kd[0], 0.0));
  assert(near(pure.cartesian_torque[0], 0.0) && near(pure.joint_assist_torque[0], 0.0));
  assert(near(pure.torque[0], -3.0));
  assert(near(pure.position[0], 0.1));
  const auto limited = compute(0.0, -20.0);
  assert(!limited.torque_saturated[0]);
  assert(near(limited.torque[0], -20.0));
  assert(!compute(-1.0).valid && !compute(2.0).valid);
  assert(!compute(std::numeric_limits<double>::quiet_NaN()).valid);
  controller.reset();
  const auto restored = compute(1.0);
  assert(near(restored.kp[0], 50.0) && near(restored.kd[0], 5.0));
}

int main() {
  {
    PvTrajectory path({0, 0.2, -0.1}, {0.2, -0.1, -0.1});
    std::vector<double> q, dq;
    path.sample(0, q, dq);
    assert(near(q[0], 0) && near(q[1], 0.2) && near(dq[0], 0));
    for (double t=0; t<path.duration_s; t+=0.01) {
      path.sample(t, q, dq);
      for (auto v : dq) assert(std::abs(v) <= 0.1 + 1e-10);
      assert(q[0] >= 0 && q[0] <= 0.2);
      assert(near(q[2], -0.1));
    }
    path.sample(path.duration_s+0.04, q, dq);
    assert(near(q[0], 0.2) && near(q[1], -0.1));
    for (auto v : dq) assert(near(v, 0));
    assert(PvTrajectory::nextDeadlineNs(0, 2'000'000) == 10'000'000);
    assert(PvTrajectory::nextDeadlineNs(10'000'000, 12'000'000) == 20'000'000);
    assert(PvTrajectory::nextDeadlineNs(10'000'000, 36'000'000) == 40'000'000);
    PvTrajectory stationary({0.2}, {0.2});
    stationary.sample(0.05, q, dq);
    assert(near(q[0], 0.2) && near(dq[0], 0));
  }
  {
    HandleController slow, fast;
    HandleParameters p;
    p.position_deadband_m = 0;
    slow.setParameters(p); fast.setParameters(p);
    slow.captureCenter(identityPose(), 0); fast.captureCenter(identityPose(), 0);
    slow.update(identityPose(), 0); fast.update(identityPose(), 0);
    auto pose = identityPose(); pose[0] = 0.01;
    fast.update(pose, 0.005);
    assert(near(slow.update(pose, 0.01).filtered_error[0],
                fast.update(pose, 0.01).filtered_error[0]));
  }
  {
    ArmSnapshot s;
    assert(!mitEntryBlockReason(s).isEmpty());
    s.connection = ConnectionState::Connected;
    s.log_ready = true;
    // Normal stopped/disabled state permits an explicitly confirmed enable.
    assert(mitEntryBlockReason(s).isEmpty());
    s.servo = ServoState::Enabled;
    assert(mitEntryBlockReason(s).isEmpty());
    for (bool ArmSnapshot::*flag : {&ArmSnapshot::exit_pending, &ArmSnapshot::low_session,
                                    &ArmSnapshot::mit_running, &ArmSnapshot::friction_batch_active}) {
      s.*flag = true;
      assert(!mitEntryBlockReason(s).isEmpty());
      s.*flag = false;
    }
    for (auto state : {SafetyState::Fault, SafetyState::EmergencyStop}) {
      s.safety = state;
      assert(!mitEntryBlockReason(s).isEmpty());
    }
    s.safety = SafetyState::Normal;
    s.controller_state = -1;
    assert(!mitEntryBlockReason(s).isEmpty());
    s.controller_state = 0;
    s.log_ready = false;
    assert(!mitEntryBlockReason(s).isEmpty());
  }
  {
    const double d=0.0005,w=0.0005,r=0.01,a=0.25;
    int zone=-1;
    assert(near(HandleController::nearAssist(d,d,w,r,a,&zone),0) && zone==0);
    assert(near(HandleController::nearAssist(d+w/2,d,w,r,a,&zone),a/2) && zone==1);
    assert(near(HandleController::nearAssist(0.003,d,w,r,a,&zone),a) && zone==2);
    const double fade=d+0.75*(r-d);
    assert(near(HandleController::nearAssist((fade+r)/2,d,w,r,a,&zone),a/2) && zone==3);
    assert(near(HandleController::nearAssist(r,d,w,r,a,&zone),0) && zone==4);
    for (double boundary : {d,d+w,fade,r})
      assert(std::abs(HandleController::nearAssist(boundary-1e-9,d,w,r,a)-
                      HandleController::nearAssist(boundary+1e-9,d,w,r,a))<1e-6);
    HandleParameters p; p.independent_gains=true; p.near_assist_enabled.fill(true);
    p.filter_alpha=1; p.position_return_breakaway_force_n=10; // Must be replaced.
    HandleController c; c.setParameters(p);
    for (double sign : {-1.0,1.0}) {
      c.captureCenter(identityPose(),0);
      auto pose=identityPose(); pose[0]=sign*0.005;
      auto out=c.update(pose,0.01);
      assert(near(out.return_assist[0],-sign*0.25));
      assert(near(out.virtual_wrench[0],out.return_assist[0]));
      pose[0]=sign*0.02;
      out=c.update(pose,0.02);
      assert(near(out.return_assist[0],0));
    }
  }
  // Independent Cartesian gains must not inherit legacy axis multipliers.
  {
    HandleController controller;
    HandleParameters p;
    p.independent_gains = true;
    p.cartesian_stiffness = {10,20,30,40,50,60};
    p.cartesian_damping = {1,2,3,4,5,6};
    p.position_axis_scale = {0.1,0.2,0.3};
    p.position_deadband_m = 0;
    p.rotation_deadband_rad = 0;
    p.filter_alpha = 1;
    controller.setParameters(p);
    controller.captureCenter(identityPose(), 0);
    controller.update(identityPose(), 0.01);
    auto pose = identityPose();
    pose[0] = pose[1] = pose[2] = 0.01;
    auto output = controller.update(pose, 0.11);
    for (int i = 0; i < 3; ++i)
      assert(near(output.virtual_wrench[i], -p.cartesian_stiffness[i]*0.01-p.cartesian_damping[i]*0.1));
    for (int axis = 0; axis < 3; ++axis) {
      controller.captureCenter(identityPose(), 0);
      controller.update(identityPose(), 0.01);
      pose = identityPose();
      pose[axis+3] = std::sin(0.005); pose[6] = std::cos(0.005);
      output = controller.update(pose, 0.11);
      assert(near(output.virtual_wrench[axis+3],
          -p.cartesian_stiffness[axis+3]*0.01-p.cartesian_damping[axis+3]*0.1));
    }
  }
  // Sparse steps separated by a pause still require wrist withdrawal.
  for (int j : {4,5}) for (int sign : {1,-1}) {
    FrictionProbe p; std::vector<double> q(6,0);
    assert(p.start(j,sign,q,0));
    for (int i=1;i<=250;++i) { p.update(q,i*.01); p.applied=p.requested; }
    q[j]=sign*.00038147; p.update(q,2.51); p.applied=p.requested;
    for (int i=252;i<=275;++i) { p.update(q,i*.01); p.applied=p.requested; }
    const double before=p.applied;
    q[j]=sign*.00076294; p.update(q,2.76);
    assert(p.active && p.requested==0 && near(p.threshold,before));
    p.applied=p.requested+p.braking;
    q[j]+=sign*.0011; p.update(q,2.77);
    const double expected=std::min(j==4?.04:.03,std::min(std::abs(before)*.5,(j==4?.3:.2)*(.11-.04)));
    assert(near(p.braking,-sign*expected));
    q[j]+=sign*.004; p.update(q,2.78);
    assert(p.outcome=="speed_guard");
  }
  for (int joint : {4,5}) for (int sign : {1,-1}) {
    FrictionProbe p; std::vector<double> q(6,0);
    assert(p.start(joint,sign,q,0));
    for (int i=1;i<=350;++i) { p.update(q,i*.01); p.applied=p.requested; }
    assert(near(p.applied,sign*.125));
    q[joint]=sign*.00038147; p.update(q,3.51); p.applied=p.requested;
    q[joint]=sign*.00076294; p.update(q,3.52);
    assert(p.threshold*sign>0 && p.requested==0); // withdraw at two forward counts
    p.cancel(); q.assign(6,0); assert(p.start(joint,sign,q,0));
    for (int i=1;i<=751;++i) {
      p.update(q,i*.01); assert(std::abs(p.requested)<=.3);
      if (p.active) p.applied=p.requested;
    }
    assert(p.outcome=="no_onset_below_limit" && near(std::abs(p.applied),.3));
  }
  {
    FrictionBatch batch;
    const std::vector<double> origin{0.599586,1.298305,-0.683032,-0.571259,0.395781,-0.011253};
    assert(batch.start(origin) && batch.withinOrigin(origin));
    auto q=origin; q[0]+=.049;
    assert(batch.withinOrigin(q));
    assert(batch.accept(.1,0)); batch.withdraw(.01); assert(batch.settled());
    assert(batch.origin()==origin); // Next sample must not reset the batch anchor.
    q[0]+=.002; assert(!batch.withinOrigin(q));
    batch.cancel(); assert(batch.start(q) && batch.withinOrigin(q));
    assert(batch.origin()==q); // A new batch captures a new pose.
    batch.cancel(); assert(!batch.start({1,2}));
    q[1]=std::numeric_limits<double>::quiet_NaN();
    assert(!batch.start(q));
  }
  for (int sign : {1,-1}) {
    auto armed = [&](FrictionProbe& p, std::vector<double>& q) {
      q.assign(6,0); assert(p.start(0,sign,q,0));
      for (int i=1;i<=300;++i) { p.update(q,i*0.01); p.applied=p.requested; }
      q[0]=sign*0.0016; p.update(q,3.01); p.applied=p.requested;
      q[0]=sign*0.00306797; p.update(q,3.02); p.applied=p.requested;
      assert(p.active);
    };
    FrictionProbe p; std::vector<double> q;
    armed(p,q); const double threshold=p.threshold;
    // Reproduce the logged one-count retreat: 0.003068 -> 0.002685.
    q[0]=sign*0.002684593;
    for (int i=303;i<=380 && p.active;++i) {
      const double previous=std::abs(p.requested);
      p.update(q,i*0.01);
      assert(std::abs(p.requested)<=previous+1e-12);
      p.applied=p.requested;
    }
    assert(p.outcome=="onset_candidate" && near(p.threshold,threshold));
    armed(p,q);
    q[0]=sign*0.002; p.update(q,3.03); assert(p.active);
    q[0]=sign*0.0027;
    for (int i=304;i<=380 && p.active;++i) p.update(q,i*0.01);
    assert(p.outcome=="onset_candidate"); // isolated below-band sample is debounced
    armed(p,q); q[0]=sign*0.002;
    p.update(q,3.03); p.update(q,3.04); assert(p.active);
    for (int i=305;i<=380 && p.active;++i) p.update(q,i*0.01);
    assert(!p.active && p.outcome=="onset_candidate");
    armed(p,q);
    for (int i=303;i<=510 && p.active;++i) {
      q[0]=sign*(i%2 ? 0.002 : 0.0027); p.update(q,i*0.01);
    }
    assert(!p.active && p.outcome=="onset_candidate"); // whole position range is within 0.0008
  }
  // Regression for J4's logged one-count toggle with jittered sampling.
  // Apply to every axis/direction; settling must not depend on 0.04 rad/s.
  for (int joint=0;joint<6;++joint) for (int sign : {1,-1}) {
    FrictionProbe p; std::vector<double> q(6,0);
    assert(p.start(joint,sign,q,0));
    for (int i=1;i<=300;++i) { p.update(q,i*.01); p.applied=p.requested; }
    q[joint]=sign*.0006; p.update(q,3.01); p.applied=p.requested;
    const double saved=p.applied;
    q[joint]=sign*.0035; p.update(q,3.02);
    double now=3.02;
    bool saw_large_derivative=false;
    for (int i=0;i<120 && p.active;++i) {
      const double dt=i%2?.012:.007;
      const double next=sign*(.0035+(i%2?0:.0003814697));
      saw_large_derivative |= std::abs(next-q[joint])/dt>.04;
      now+=dt; q[joint]=next;
      p.applied=p.requested+p.braking; p.update(q,now);
    }
    assert(saw_large_derivative && !p.active && p.outcome=="onset_candidate");
    assert(near(p.threshold,saved) && p.requested==0 && p.braking==0);
  }
  // Wrist regression: withdraw before 0.003 rad, cap the brake relative to
  // excitation, and never reverse the brake after a rebound (both directions).
  for (int joint : {4,5}) for (int sign : {1,-1}) {
    FrictionProbe p; std::vector<double> q(6,0);
    assert(p.start(joint,sign,q,0));
    for (int i=1;i<=150;++i) { p.update(q,i*.01); p.applied=p.requested; }
    q[joint]=sign*.00038; p.update(q,1.51); p.applied=p.requested;
    const double before=p.applied;
    q[joint]=sign*.0020; p.update(q,1.52);
    assert(near(p.threshold,before) && p.requested==0);
    assert(p.braking*sign<=0 && std::abs(p.braking)<=std::min(joint==4?.04:.03,std::abs(before)*.5));
    q[joint]=sign*.0013; p.update(q,1.53);
    assert(p.active && p.braking==0); // Reversal ends braking, not a new opposite pulse.
    q[joint]=sign*.0021; p.update(q,1.54);
    assert(p.braking==0 && p.requested==0);
    for (int i=155;i<=220 && p.active;++i) { p.applied=0; p.update(q,i*.01); }
    assert(p.outcome=="onset_candidate" && near(p.threshold,before));
    q.assign(6,0); assert(p.start(joint,sign,q,0));
    for (int i=1;i<=200;++i) {
      q[joint]=sign*(i%2)*.000383; p.update(q,i*.01); p.applied=p.requested;
    }
    assert(p.active && p.threshold==0 && p.braking==0); // Encoder-count noise is not onset.
    q[joint]+=sign*.004; p.update(q,2.01);
    assert(p.outcome=="speed_guard"); // Raw protection is never filtered away.
  }
  // All axes share fast withdrawal and bounded post-onset braking. Threshold
  // must remain the last pre-braking acknowledged excitation, not the brake.
  for (int joint=0;joint<6;++joint) for (int sign : {1,-1}) {
    FrictionProbe probe; std::vector<double> q(6,0);
    assert(probe.start(joint,sign,q,0));
    for (int i=1;i<=700;++i) { probe.update(q,i*0.01); probe.applied=probe.requested; }
    assert(near(probe.applied,sign*std::min(FrictionProbe::axisLimit(joint),6*FrictionProbe::rampRate(joint))));
    q[joint]=sign*0.00038; probe.update(q,7.01); probe.applied=probe.requested;
    const double threshold=probe.applied;
    q[joint]=sign*0.0032; probe.update(q,7.02);
    assert(probe.active && near(probe.threshold,threshold));
    assert(std::abs(probe.requested)<0.71 && probe.braking*sign<0);
    for (int i=703;i<=704;++i) {
      probe.applied=probe.requested+probe.braking;
      q[joint]+=sign*0.0015; probe.update(q,i*0.01);
      assert(std::abs(probe.braking)<=0.8 && probe.braking*sign<=0);
    }
    assert(near(probe.requested,0));
    for (int i=705;i<=720;++i) {
      probe.applied=probe.requested+probe.braking; probe.update(q,i*0.01);
      assert(probe.active); // Confirmation alone must not end a moving test.
    }
    for (int i=721;i<=790 && probe.active;++i) {
      probe.applied=probe.requested+probe.braking; probe.update(q,i*0.01);
    }
    assert(probe.outcome=="onset_candidate" && near(probe.threshold,threshold));
    assert(near(probe.applied,0) && probe.braking==0);
  }
  {
    FrictionProbe probe; std::vector<double> q(6,0);
    assert(probe.start(3,1,q,0));
    for (int i=1;i<=300;++i) { probe.update(q,i*0.01); probe.applied=probe.requested; }
    q[3]=0.002; probe.update(q,3.01); probe.applied=probe.requested;
    q[3]=0.0035; probe.update(q,3.02);
    // Remain within displacement/speed guards, but never settle. Must not
    // report a candidate or allow the batch to apply the next excitation.
    for (int i=303;i<=510 && probe.active;++i) {
      q[3]=(i/6)%2 ? 0.0035 : 0.0047;
      probe.applied=probe.requested+probe.braking; probe.update(q,i*0.01);
      assert(std::abs(probe.braking)<=0.8);
    }
    assert(!probe.active && probe.outcome=="braking_timeout");
  }
  // Regression: withdraw during the 100 ms onset confirmation, in both
  // directions, and do not reapply the saved threshold on batch handoff.
  for (int sign : {1,-1}) {
    FrictionProbe probe;
    std::vector<double> q(6,0);
    assert(probe.start(0,sign,q,0));
    for (int i=1;i<=550;++i) { probe.update(q,i*0.01); probe.applied=probe.requested; }
    q[0]=sign*0.0016; probe.update(q,5.51); probe.applied=probe.requested;
    const double onset_torque=probe.applied;
    q[0]=sign*0.0032; probe.update(q,5.52);
    assert(probe.active && near(probe.threshold,onset_torque));
    assert(std::abs(probe.requested)<std::abs(onset_torque));
    double previous=std::abs(probe.requested);
    probe.applied=probe.requested;
    for (int i=553;i<=640 && probe.active;++i) {
      probe.update(q,i*0.01);
      assert(std::abs(probe.requested)<=previous+1e-12);
      assert(probe.requested*sign>=0);
      previous=std::abs(probe.requested);
      probe.applied=probe.requested;
    }
    assert(!probe.active && probe.outcome=="onset_candidate");
    assert(near(probe.threshold,onset_torque) && near(probe.applied,0));
    FrictionBatch batch;
    assert(batch.start());
    if (sign<0) { assert(batch.accept(0.5,0)); batch.withdraw(0.01); assert(batch.settled()); }
    assert(batch.accept(probe.threshold,probe.applied));
    assert(near(batch.withdraw(0.01),0) && batch.phase==FrictionBatch::Phase::Settling);
  }
  {
    FrictionBatch batch;
    assert(batch.start() && !batch.start());
    assert(!batch.accept(-0.1,0));
    assert(!batch.accept(5.01,0));
    assert(!batch.accept(std::numeric_limits<double>::quiet_NaN(),0));
    for (int i=0;i<36;++i) {
      assert(batch.joint()==i/6 && batch.direction()==(i%2==0?1:-1));
      assert(batch.repetition()==(i%6)/2+1);
      assert(batch.accept(batch.direction()*0.5,batch.direction()*0.2));
      assert(!batch.accept(0.5,0));
      assert(batch.phase==FrictionBatch::Phase::Releasing);
      assert(!batch.settled());
      double previous=std::abs(batch.release_torque);
      while (batch.phase==FrictionBatch::Phase::Releasing) {
        const double output=batch.withdraw(0.01);
        assert(std::abs(output)<=previous && (output==0 || output*(i%2==0?1:-1)>0));
        assert(previous-std::abs(output)<=0.100001); previous=std::abs(output);
      }
      assert(batch.phase==FrictionBatch::Phase::Settling && batch.release_torque==0);
      assert(batch.settled());
      if (i<35) assert(batch.phase==FrictionBatch::Phase::Probing);
    }
    assert(!batch.active() && batch.thresholds.size()==36);
    assert(batch.start() && batch.thresholds.empty());
    batch.cancel(); assert(!batch.active() && !batch.accept(0.5,0));
  }
  {
    FrictionProbe probe;
    std::vector<double> q(6,0);
    assert(probe.start(0,1,q,0));
    for (int i=1;i<=150;++i) { probe.update(q,i*0.01); probe.applied=probe.requested; }
    assert(probe.active && near(probe.requested,0.1));
    for (int i=151;i<=170;++i) {
      q[0]=std::min(0.004,0.001*(i-150));
      probe.update(q,i*0.01);
    }
    for (int i=171;i<=240 && probe.active;++i) probe.update(q,i*0.01);
    assert(!probe.active && probe.outcome=="onset_candidate" && near(probe.threshold,0.1));
    q.assign(6,0); assert(probe.start(1,-1,q,0));
    q[1]=0.002; probe.update(q,0.01);
    assert(!probe.active && probe.outcome=="baseline_drift");
    q.assign(6,0); assert(probe.start(1,1,q,0));
    probe.update(q,0.2); assert(!probe.active);
    assert(probe.start(1,1,q,0));
    for (int i=1;i<=2651;++i) {
      probe.update(q,i*0.01);
      assert(std::abs(probe.requested)<=5.0);
      if (probe.active) probe.applied=probe.requested;
    }
    assert(!probe.active && probe.outcome=="no_onset_below_limit");
    assert(near(probe.applied,5));
    q.assign(6,0); assert(probe.start(0,1,q,0));
    for (int i=1;i<=110;++i) probe.update(q,i*0.01);
    for (int i=111;i<=116;++i) { q[1]+=0.001; probe.update(q,i*0.01); }
    assert(!probe.active && probe.outcome=="displacement_guard");
    q.assign(6,0); assert(probe.start(0,1,q,0));
    q[0]=0.004; probe.update(q,0.01);
    assert(!probe.active && probe.outcome=="speed_guard");
    MitHandleController controller;
    MitSafetyParameters safety; safety.torque_limit_nm=10;
    safety.interaction_torque_limit_nm=2; safety.torque_rate_limit_nm_s=100;
    controller.setSafetyParameters(safety);
    controller.setJointGains({80},{5});
    HandleOutput h; h.valid=true; h.dt=0.01; h.virtual_wrench[0]=100;
    auto run=[&](double extra) { return controller.compute(h,{0},{0},{4},{1,0,0,0,0,0},6,1,0,{extra}); };
    assert(near(run(0).torque[0],4));
    auto c=run(0.2);
    assert(c.valid && near(c.torque[0],4.2) && near(c.kp[0],0) && near(c.kd[0],0));
    c=run(3); c=run(3);
    assert(near(c.interaction_torque[0],3) && near(c.torque[0],7));
    for (int i=0;i<10;++i) c=controller.compute(h,{0},{0},{4},{1,0,0,0,0,0},6,1,0,{5},5);
    assert(c.valid && near(c.torque[0],9) && near(c.interaction_torque[0],5));
    c=controller.compute(h,{0},{0},{4},{1,0,0,0,0,0},6,1,1,{5},5);
    assert(c.valid && near(c.interaction_torque[0],105) && near(c.torque[0],109));
    c=controller.compute(h,{0},{0},{4},{1,0,0,0,0,0},6,1,0,{6},6);
    assert(c.valid && near(c.interaction_torque[0],6) && near(c.torque[0],10));
    for (int i=0;i<10;++i) c=run(5);
    assert(near(c.interaction_torque[0],5));
    for (int i=0;i<10;++i) c=controller.compute(h,{0},{0},{8},{1,0,0,0,0,0},6,1,0,{5},5);
    assert(c.valid && !c.torque_saturated[0] && near(c.torque[0],13));
  }
  {
    FrictionBatch batch;
    assert(batch.start());
    for (int i=0;i<36;++i) {
      const int sign=batch.direction();
      assert(!batch.unidentified(sign*5.01));
      assert(batch.unidentified(sign*5.0));
      assert(batch.identified.back()==0 && batch.thresholds.back()==0);
      double previous=5;
      while (batch.phase==FrictionBatch::Phase::Releasing) {
        const double torque=batch.withdraw(0.01);
        assert(torque*sign>=0 && std::abs(torque)<=previous);
        previous=std::abs(torque);
      }
      assert(batch.settled());
    }
    assert(!batch.active() && batch.index()==36);
  }
  {
    MitHandleController controller;
    MitSafetyParameters limits;
    limits.torque_limit_nm = 10;
    limits.interaction_torque_limit_nm = 1;
    limits.torque_rate_limit_nm_s = 10000;
    controller.setSafetyParameters(limits);
    controller.setJointGains({0}, {0});
    controller.setJointCenterAssist({1}, {10}, {0}, 0.8, 0.3);
    HandleOutput h; h.valid = true; h.dt = 0.01;
    auto run = [&](double g) { return controller.compute(h, {0}, {0}, {g}, {1,0,0,0,0,0}, 6, 1); };
    assert(near(run(4).torque[0], 4));
    h.virtual_wrench[0] = 5;
    auto c = run(4);
    assert(c.valid && near(c.interaction_torque[0], 5.8) && near(c.torque[0], 9.8));
    assert(near(c.kp[0], 0) && near(c.kd[0], 0));
    h.virtual_wrench[0] = -5;
    c = run(4);
    assert(near(c.interaction_torque[0], -4.2) && near(c.torque[0], -0.2));
    h.virtual_wrench[0] = 5;
    c = run(9.5);
    assert(!c.torque_saturated[0] && near(c.torque[0], 15.3));
    limits.interaction_torque_limit_nm = -1;
    controller.setSafetyParameters(limits);
    assert(run(4).valid);
  }
  {
    UrdfGravity gravity;
    QString reason;
    const auto path=QFileInfo(TEST_CONFIG_PATH).absoluteDir().absoluteFilePath("../../carm_a3t/carm_a3t/urdf/carm_a3t.urdf");
    assert(gravity.load(path,&reason));
    const auto actual=gravity.compute({0.013997077941894531,0.9140472412109375,-0.530059814453125,
                                      -0.3751811981201172,-0.14362525939941406,-0.00743865966796875});
    const std::vector<double> expected{0,-0.7592122320493768,-8.12576333308909,-2.078851709505703,
                                       -0.0009861317083683409,0.0003053804960136009};
    assert(actual.size()==6);
    for (size_t i=0;i<6;++i) assert(near(actual[i],expected[i],1e-9));
    const auto tilted=gravity.compute({0.3,1.5,-1.2,0.2,0.4,-0.5});
    const std::vector<double> reference{0,-7.089258419682699,-7.947560749438061,-1.9015623410240348,
                                        0.14693379539103477,0.0001840439300709925};
    for (size_t i=0;i<6;++i) assert(near(tilted[i],reference[i],1e-9));
    const auto zero=gravity.compute(std::vector<double>(6,0));
    assert(near(zero[1],4.276235798798275,1e-9));
    assert(gravity.compute({}).empty());
    assert(gravity.compute({0,0,0,0,0,std::numeric_limits<double>::quiet_NaN()}).empty());
    assert(!gravity.load(path+".missing",&reason));
    assert(gravity.compute(std::vector<double>(6,0)).empty());
  }
  testGravityOnlyTransition();
  {
    UrdfGravity gravity;
    QString reason;
    const auto path=QFileInfo(TEST_CONFIG_PATH).absoluteDir().absoluteFilePath("gravity_gripper_can.urdf");
    assert(gravity.load(path,&reason));
    const auto torque=gravity.compute({0.01438,0.91213,-0.533493,-0.375944,-0.143625,-0.006676});
    const std::vector<double> expected{0,-0.11183522922869005,-6.890280706184223,
        -1.558701495937331,-0.00018331594341196116,-0.000026898443063627015};
    assert(torque.size()==6);
    for (size_t i=0;i<6;++i) assert(near(torque[i],expected[i],1e-9));
  }
  testReturnForceAndAssistRegression();
  {
    UrdfGravity baseline;
    QString reason;
    const auto path=QFileInfo(TEST_CONFIG_PATH).absoluteDir().absoluteFilePath("gravity_gripper_can.urdf");
    assert(baseline.load(path,&reason));
    const std::vector<double> q{0.01438,0.91213,-0.533493,-0.375944,-0.143625,-0.006676};
    const auto original=baseline.compute(q);
    auto candidate=baseline;
    std::vector<double> d{0.00029498147876614716,-0.02,0,0.007327879583522159,
      0.006281505679824627,0,0.011289359903009433,-0.0009533468625387454,0,
      0.09999999999970766,0.005085385158052422,-0.00494248215420607,0.026293492468900507};
    assert(candidate.applyGravityFit(d,&reason));
    // Independent Python regressor reference; J2 must not be masked out.
    assert(near(candidate.compute(q)[1]-original[1],-0.352546097866077,1e-9));
    assert(baseline.compute(q)==original);
    const auto before=candidate.compute(q);
    d[0]=1;
    assert(!candidate.applyGravityFit(d,&reason));
    assert(candidate.compute(q)==before);
    assert(!candidate.applyGravityFit({},&reason));
  }
  {
    MitStartupHold hold;
    hold.reset({0.0});
    assert(near(hold.update(0.0, {0.004}, {0.0}), 1.0));
    std::vector<double> q{0.004};
    hold.apply(1.0, q);
    assert(near(q[0], 0.0));
    assert(near(hold.update(0.5, {0.004}, {0.0}), 1.0));
    const double weight = hold.update(0.75, {0.004}, {0.0});
    assert(near(weight, 0.5));
    q = {0.004}; hold.apply(weight, q);
    assert(near(q[0], 0.002));
    assert(near(hold.update(1.0, {0.004}, {0.0}), 0.0));
    assert(!hold.active());
    hold.reset({0.0});
    assert(hold.update(0.0, {0.02}, {0.0}) == 1.0);
    assert(hold.update(2.0, {0.02}, {0.0}) == 0.0);
    hold.reset({0.0});
    assert(hold.update(0.0, {0.031}, {0.0}) < 0.0);
    assert(hold.update(0.0, {0.0}, {0.174}) == 1.0);
    assert(hold.update(0.0, {0.0}, {std::numeric_limits<double>::quiet_NaN()}) < 0.0);
    assert(hold.update(0.0, {}, {}) < 0.0);
    assert(hold.update(0.0, {std::numeric_limits<double>::quiet_NaN()}, {0.0}) < 0.0);
    hold.reset({0.0});
    assert(hold.update(0.0, {0.0}, {0.0}) == 1.0);
    assert(hold.update(0.4, {0.01}, {0.0}) == 1.0);
    assert(hold.update(0.5, {0.0}, {0.0}) == 1.0);
    assert(hold.update(0.6, {0.0}, {0.0}) < 1.0);
    hold.reset();
    assert(!hold.active());
  }
  testInvalidPose();
  testConfiguration();
  testHandleDeadbandAndWrench();
  testSmoothCartesianBreakawayForce();
  testCartesianAxisScaling();
  testMitPerJointTorqueLimit();
  testShortestQuaternionRotation();
  testBaseFrameRotation();
  testSquareJacobianLayout();
  testMitMappingAndLimits();
  testMitMotionSafety();
  return 0;
}
