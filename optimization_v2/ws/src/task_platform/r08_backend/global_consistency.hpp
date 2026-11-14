#pragma once
#include <Eigen/Geometry>
namespace r08 {
inline double correctionDisplacement(const Eigen::Isometry3d& next,const Eigen::Isometry3d& previous,const Eigen::Vector3d& current_odom_position){return (next*current_odom_position-previous*current_odom_position).norm();}
inline double antennaDisagreement(const Eigen::Isometry3d& next,const Eigen::Isometry3d& observed_odom,const Eigen::Vector3d& lever,const Eigen::Vector3d& observed_antenna){return (next*observed_odom*lever-observed_antenna).head<2>().norm();}
}
