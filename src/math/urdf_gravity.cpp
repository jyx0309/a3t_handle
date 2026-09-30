#include "math/urdf_gravity.hpp"
#include <QCryptographicHash>
#include <QDomDocument>
#include <QFile>
#include <QRegularExpression>
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
double number(const QString& text) {
  bool ok=false; const double value=text.toDouble(&ok);
  if (!ok || !std::isfinite(value)) throw std::runtime_error("Invalid URDF number");
  return value;
}
Eigen::Vector3d xyz(const QString& text) {
  const auto parts=text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
  if (parts.size()!=3) throw std::runtime_error("Invalid URDF vector");
  return {number(parts[0]),number(parts[1]),number(parts[2])};
}
Eigen::Isometry3d transform(const QDomElement& origin) {
  Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
  t.translation()=xyz(origin.attribute("xyz","0 0 0"));
  const auto r=xyz(origin.attribute("rpy","0 0 0"));
  t.linear()=(Eigen::AngleAxisd(r.z(),Eigen::Vector3d::UnitZ()) *
              Eigen::AngleAxisd(r.y(),Eigen::Vector3d::UnitY()) *
              Eigen::AngleAxisd(r.x(),Eigen::Vector3d::UnitX())).toRotationMatrix();
  return t;
}
}

bool UrdfGravity::applyGravityFit(const std::vector<double>& d, QString* reason) {
  if (d.size()!=13) { *reason="Expected 13 gravity-fit parameters"; return false; }
  for (size_t i=0;i<d.size();++i) {
    double bound=i<9?0.02:(i==9?0.1:0.03);
    if (!std::isfinite(d[i]) || std::abs(d[i])>bound+1e-12) {
      *reason="Gravity fit outside exploratory bounds"; return false;
    }
  }
  auto candidate=links_;
  const std::array<QString,4> names={"link2","link3","link4","gripper_base"};
  for (size_t n=0;n<names.size();++n) {
    auto l=std::find_if(candidate.begin(),candidate.end(),[&](const Link& v){return v.name==names[n];});
    if (l==candidate.end()) { *reason="Missing gravity fit link"; return false; }
    if (n<3) l->com+=Eigen::Vector3d(d[3*n],d[3*n+1],d[3*n+2]);
    else {
      double mass=l->mass+d[9];
      if (mass<=0) { *reason="Invalid fitted mass"; return false; }
      l->com+=l->mass/mass*Eigen::Vector3d(d[10],d[11],d[12]); l->mass=mass;
    }
  }
  links_=std::move(candidate);
  return true; // hash remains source-file hash; log fit parameters separately.
}

bool UrdfGravity::load(const QString& path, QString* reason) {
  links_.clear(); joints_.clear(); root_=-1; sha256_.clear();
  try {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) throw std::runtime_error("Cannot read gravity URDF");
    const auto bytes=file.readAll();
    QDomDocument doc;
    if (!doc.setContent(bytes) || doc.documentElement().tagName()!="robot")
      throw std::runtime_error("Invalid URDF XML");
    const auto robot=doc.documentElement();
    auto index=[&](const QString& name) {
      for (size_t i=0;i<links_.size();++i) if (links_[i].name==name) return int(i);
      return -1;
    };
    for (auto l=robot.firstChildElement("link"); !l.isNull(); l=l.nextSiblingElement("link")) {
      const auto inertia=l.firstChildElement("inertial");
      const auto name=l.attribute("name");
      if (name.isEmpty() || index(name)>=0 || inertia.isNull()) throw std::runtime_error("Missing/duplicate link inertial");
      const double mass=number(inertia.firstChildElement("mass").attribute("value"));
      if (mass<0) throw std::runtime_error("Negative mass");
      links_.push_back({name,mass,transform(inertia.firstChildElement("origin")).translation()});
    }
    std::vector<int> parents(links_.size(),-1);
    std::array<bool,6> found{};
    for (auto j=robot.firstChildElement("joint"); !j.isNull(); j=j.nextSiblingElement("joint")) {
      const int parent=index(j.firstChildElement("parent").attribute("link"));
      const int child=index(j.firstChildElement("child").attribute("link"));
      if (parent<0 || child<0 || parents[child]!=-1) throw std::runtime_error("Invalid joint tree");
      parents[child]=parent;
      int joint_index=-1;
      const auto name=j.attribute("name"), type=j.attribute("type");
      Eigen::Vector3d axis=Eigen::Vector3d::UnitZ();
      if (type=="revolute") {
        for (int i=0;i<6;++i) if (name==QString("joint%1").arg(i+1)) joint_index=i;
        if (joint_index<0 || found[joint_index] || !j.firstChildElement("mimic").isNull())
          throw std::runtime_error("Expected independent joint1..6");
        found[joint_index]=true;
        axis=xyz(j.firstChildElement("axis").attribute("xyz"));
        if (axis.norm()<1e-12) throw std::runtime_error("Zero joint axis");
        axis.normalize();
      } else if (type=="prismatic") {
        // This model's two symmetric fingers are evaluated at zero opening.
        if (name!="gripper_right_joint" && name!="gripper_left_joint")
          throw std::runtime_error("Unsupported prismatic joint");
        const auto mimic=j.firstChildElement("mimic");
        if (!mimic.isNull() && (mimic.attribute("joint")!="gripper_right_joint" ||
            number(mimic.attribute("offset","0"))!=0)) throw std::runtime_error("Unsupported finger mimic");
      } else if (type!="fixed") throw std::runtime_error("Unsupported joint type");
      joints_.push_back({parent,child,joint_index,transform(j.firstChildElement("origin")),axis});
    }
    for (bool yes:found) if (!yes) throw std::runtime_error("Missing arm joint");
    for (size_t i=0;i<parents.size();++i) if (parents[i]==-1) {
      if (root_!=-1) throw std::runtime_error("Multiple roots");
      root_=int(i);
    }
    if (root_<0 || links_[root_].name!="base_link") throw std::runtime_error("Expected base_link root");
    if (compute(std::vector<double>(6,0)).size()!=6) throw std::runtime_error("Disconnected/cyclic model");
    sha256_=QCryptographicHash::hash(bytes,QCryptographicHash::Sha256).toHex();
    return true;
  } catch (const std::exception& e) {
    *reason=QString::fromUtf8(e.what()); root_=-1; return false;
  }
}

std::vector<double> UrdfGravity::compute(const std::vector<double>& q) const {
  if (root_<0 || q.size()!=6) return {};
  for (double value:q) if (!std::isfinite(value)) return {};
  std::vector<Eigen::Isometry3d> poses(links_.size(),Eigen::Isometry3d::Identity());
  std::vector<std::array<bool,6>> ancestors(links_.size());
  std::vector<bool> ready(links_.size(),false); ready[root_]=true;
  std::array<Eigen::Vector3d,6> axes, pivots;
  size_t remaining=joints_.size();
  while (remaining) {
    bool progress=false;
    for (const auto& joint:joints_) if (ready[joint.parent] && !ready[joint.child]) {
      auto t=poses[joint.parent]*joint.origin;
      ancestors[joint.child]=ancestors[joint.parent];
      if (joint.index>=0) {
        axes[joint.index]=t.linear()*joint.axis; pivots[joint.index]=t.translation();
        ancestors[joint.child][joint.index]=true;
        t.rotate(Eigen::AngleAxisd(q[joint.index],joint.axis));
      }
      poses[joint.child]=t; ready[joint.child]=true; --remaining; progress=true;
    }
    if (!progress) return {};
  }
  std::vector<double> torque(6,0.0);
  for (size_t link=0;link<links_.size();++link) {
    if (!ready[link]) return {};
    const Eigen::Vector3d point=poses[link]*links_[link].com;
    const Eigen::Vector3d force(0,0,-9.81*links_[link].mass);
    for (int i=0;i<6;++i) if (ancestors[link][i])
      torque[i]-=axes[i].cross(point-pivots[i]).dot(force);
  }
  for (double value:torque) if (!std::isfinite(value)) return {};
  return torque;
}
