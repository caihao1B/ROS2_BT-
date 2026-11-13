#include "pose_jump17.hpp"
#include <cassert>
#include <iostream>
#include <limits>
using T=Eigen::Isometry3d;
int main() {
  int checked=0;
  for (double distance : {0., 20., 100., 1000.}) {
    T odom=T::Identity();odom.translation()<<distance,7.,0.;
    T global=T::Identity();global.translation()<<30.,40.,0.;
    const T current=global*odom.inverse();
    T candidate_pose=global;
    candidate_pose.linear()=Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const T candidate=candidate_pose*odom.inverse();
    assert(!s17::pose_jump(current,candidate,odom,.15,.035));++checked;
    assert(s17::gnss_map_residual(candidate,odom,global.translation())<1e-10);++checked;
    candidate_pose.translation().x()+=.151;
    assert(s17::pose_jump(current,candidate_pose*odom.inverse(),odom,.15,.035));++checked;
    assert(s17::gnss_map_residual(candidate_pose*odom.inverse(),odom,global.translation())>.15);++checked;
    candidate_pose.translation()=global.translation();
    candidate_pose.linear()=Eigen::AngleAxisd(.036,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    assert(s17::pose_jump(current,candidate_pose*odom.inverse(),odom,.15,.035));++checked;
    // Express the identical motion in another odometry coordinate system.
    T change=T::Identity();change.translation()<<-300.,200.,1.;
    change.linear()=Eigen::AngleAxisd(.7,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    assert(s17::pose_jump(current*change.inverse(),candidate*change.inverse(),change*odom,.15,.035)==s17::pose_jump(current,candidate,odom,.15,.035));++checked;
  }
  T invalid=T::Identity();invalid.translation().x()=std::numeric_limits<double>::quiet_NaN();
  assert(s17::pose_jump(T::Identity(),invalid,T::Identity(),.15,.035));++checked;
  std::cout<<"{\"passed\":true,\"checks\":"<<checked<<"}\n";
}
