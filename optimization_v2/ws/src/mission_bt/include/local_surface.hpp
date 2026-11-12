#pragma once
#include <Eigen/Eigenvalues>
#include <vector>
#include <algorithm>
#include "slope_observability.hpp"
namespace s14 {
struct LocalSurface {
 bool valid=false;
 Eigen::Vector3d mean=Eigen::Vector3d::Zero(),normal=Eigen::Vector3d(0,0,1);
 double slope=0.,rough=.1,step=0.,sigma=90.;
 size_t support=0;
};
// A terrain cell describes geometry AT that cell. The Nav2 footprint and
// inflation layer handle spatial clearance; a distant tree must not become
// a step in every cell within the 7x7 fitting window.
inline LocalSurface localSurface(const std::vector<Eigen::Vector3d>& neighbors,
 const Eigen::Vector3d& center,double residual_limit,double height_sigma) {
 LocalSurface result;
 std::vector<Eigen::Vector3d> active=neighbors;
 for(int iteration=0;iteration<4;++iteration) {
  if(active.size()<8 || active.size()*2<neighbors.size())return result;
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();
  for(const auto& p:active){if(!p.allFinite())return result;mean+=p;}
  mean/=active.size();Eigen::Matrix3d sum=Eigen::Matrix3d::Zero();
  for(const auto& p:active)sum+=(p-mean)*(p-mean).transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(sum/active.size());
  if(eigen.info()!=Eigen::Success)return result;
  Eigen::Vector3d normal=eigen.eigenvectors().col(0);
  if(normal.z()<0)normal=-normal;
  if(normal.z()<.5)return result;
  std::vector<Eigen::Vector3d> next;
  for(const auto& p:neighbors)
   if(std::abs((p-mean).dot(normal)/normal.z())<=residual_limit)next.push_back(p);
  if(next.size()<8 || next.size()*2<neighbors.size())return result;
  bool unchanged=next.size()==active.size();
  if(unchanged)for(size_t i=0;i<next.size();++i)
   if((next[i]-active[i]).squaredNorm()>1e-20){unchanged=false;break;}
  // Bounded fit: nonconvergence is unknown, never a free-space claim.
  if(unchanged) {
   result.valid=true;result.mean=mean;result.normal=normal;result.support=active.size();
   result.slope=std::acos(std::clamp(normal.z(),0.,1.))*180/M_PI;
   result.rough=std::sqrt(std::max(0.,eigen.eigenvalues()[0]));
   result.sigma=slopeSigma(sum,height_sigma);
   result.step=std::abs((center-mean).dot(normal)/normal.z());
   return result;
  }
  active=std::move(next);
 }
 return result;
}
}
