#include "acceptance_monitor.hpp"
#include "pose_jump17.hpp"
#include <cassert>
#include <iostream>
int main(){
 s17::json evidence={{"clock",{0.,1.,1799.9}},{"contacts",{{"vehicle::wheel|ground::body::ground",100}}},{"tool_stopped",true},{"limit_s",1800.}};
 assert(s17::monitor(evidence)["passed"]==true);
 evidence["clock"]={0.,2.,1.};assert(s17::monitor(evidence)["passed"]==false);
 evidence["clock"]={0.,1800.001};assert(s17::monitor(evidence)["passed"]==false);
 evidence["clock"]={0.,10.};evidence["contacts"]["vehicle::wheel|low_obstacle::body::collision"]=1;
 assert(s17::monitor(evidence)["passed"]==false);
 evidence["contacts"]=s17::json::object();evidence["tool_stopped"]=false;assert(s17::monitor(evidence)["passed"]==false);
 bool rejected=false;try{evidence["clock"]=s17::json::array();s17::monitor(evidence);}catch(const std::exception&){rejected=true;}assert(rejected);
 // The same 0.01 rad correction has a large transform-origin translation far
 // from the odometry origin, despite leaving the actual robot position fixed.
 Eigen::Isometry3d odom=Eigen::Isometry3d::Identity();odom.translation().x()=100.;
 Eigen::Isometry3d current=Eigen::Isometry3d::Identity(),global=current*odom;
 global.linear()=Eigen::AngleAxisd(.01,Eigen::Vector3d::UnitZ()).toRotationMatrix();
 Eigen::Isometry3d next=global*odom.inverse();
 assert((next.translation()-current.translation()).norm()>.15);
 assert(!s17::pose_jump(current,next,odom,.15,.035));
 global.translation().x()+=.151;
 assert(s17::pose_jump(current,global*odom.inverse(),odom,.15,.035));
 std::cout<<"9 C++ checks passed: six independent evidence checks and three coordinate-origin checks\n";
}
