// Teaching example only: production ROS state machine is global_pose_manager.cpp.
#include <Eigen/Geometry>
#include <cassert>
#include <iostream>
using T=Eigen::Isometry3d;
T se2(double x,double y,double yaw){T t=T::Identity();t.translation()<<x,y,0;t.linear()=Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();return t;}
int main(){
  T odom=se2(1,0,.1),observed=se2(4,2,.2),correction=observed*odom.inverse();
  assert((correction*odom).matrix().isApprox(observed.matrix(),1e-12));
  int confirmations=0;bool allowed=false;
  for(bool valid:{true,false,true,true,true}){confirmations=valid?confirmations+1:0;allowed=confirmations>=3;std::cout<<"valid="<<valid<<" count="<<confirmations<<" allowed="<<allowed<<"\n";}
  assert(allowed);allowed=false; // Expired evidence always closes permission.
  assert(!allowed);
  std::cout<<"T_map_odom translation="<<correction.translation().transpose()<<"; expired evidence STOP\n";
}
