#pragma once
#include <Eigen/Geometry>
#include <QString>
#include <vector>

// Static gravity only. No friction or SDK calls; fixed base, gravity along -Z.
class UrdfGravity {
 public:
  bool load(const QString& path, QString* reason);
  std::vector<double> compute(const std::vector<double>& q) const;
  // Exploratory gravity fit on a copy, not a validated dynamics model.
  bool applyGravityFit(const std::vector<double>& delta, QString* reason);
  QString sha256() const { return sha256_; }
 private:
  struct Link { QString name; double mass; Eigen::Vector3d com; };
  struct Joint {
    int parent, child, index;
    Eigen::Isometry3d origin;
    Eigen::Vector3d axis;
  };
  std::vector<Link> links_;
  std::vector<Joint> joints_;
  int root_{-1};
  QString sha256_;
};
