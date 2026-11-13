#pragma once
#include <Eigen/Geometry>

namespace s17 {
// Compare the robot at one physical epoch, not the translation of a freely
// chosen odometry origin. A change of that origin must not alter admission.
inline bool pose_jump(const Eigen::Isometry3d& current,
                      const Eigen::Isometry3d& candidate,
                      const Eigen::Isometry3d& odometry,
                      double translation_limit, double rotation_limit) {
  if (!current.matrix().allFinite() || !candidate.matrix().allFinite() ||
      !odometry.matrix().allFinite()) return true;
  const Eigen::Vector3d displacement = (candidate * odometry).translation() -
                                      (current * odometry).translation();
  const double angle = Eigen::AngleAxisd(
      current.linear().transpose() * candidate.linear()).angle();
  return displacement.norm() > translation_limit || angle > rotation_limit;
}
inline double gnss_map_residual(const Eigen::Isometry3d& map_correction,
                               const Eigen::Isometry3d& odometry_at_fix,
                               const Eigen::Vector3d& observed_base_position) {
  return ((map_correction * odometry_at_fix).translation() -
          observed_base_position).head<2>().norm();
}
}  // namespace s17
