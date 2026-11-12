#pragma once
#include <Eigen/Eigenvalues>
#include <terrain_perception/risk.hpp>
namespace s14 {
// For z=ax+by+c, covariance(a,b) = sigma_z^2 (X'X)^-1.
// A narrow scan ring does not measure two independent ground gradients.
inline double slopeSigma(const Eigen::Matrix3d& centered_sum, double sigma_z) {
 Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(centered_sum.topLeftCorner<2,2>());
 const double baseline=eigen.eigenvalues()[0];
 return baseline>1e-12 ? std::atan(sigma_z/std::sqrt(baseline))*180/M_PI : 90.;
}
inline int unobservableSlopeRisk(double height,double reference_slope,const s09::Profile& profile) {
 // Still stop for directly measured protrusions, negative steps or a steep
 // verified reference. Absence of a supported local plane never means free.
 return s09::risk(reference_slope,std::abs(height),.01,height,false,profile)==100 ? 100 : -1;
}
}
