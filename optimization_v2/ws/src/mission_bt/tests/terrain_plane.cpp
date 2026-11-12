#include <cassert>
#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include "step_reference.hpp"
#include "collision_volume.hpp"
#include "slope_observability.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <terrain_perception/risk.hpp>
int main(int argc,char** argv){
 using V=Eigen::Vector3d;s09::Profile profile{.04,5.,.035,.22,.3};V normal(0,0,1),mean(0,0,-.004),sample(.15,0,-.015);
 double corrected=std::abs(sample.z()-s14::planeHeight(mean,normal,sample));
 double old=std::abs(sample.z()-.0146);
 assert(s09::risk(.2,old,.0048,.0146,true,profile)==100);
 assert(s09::risk(.2,corrected,.0048,.0146,true,profile)==0);
 V raised(.15,0,.06);double step=std::abs(raised.z()-s14::planeHeight(V(0,0,.01),normal,raised));
 assert(s09::risk(.2,step,.01,.06,true,profile)==100);
 assert(s09::risk(5.,0.,0.,0.,true,profile)==100);
 assert(s14::inCollisionVolume(.06,1.1));
 assert(s14::inCollisionVolume(-.2,1.1));
 assert(s14::inCollisionVolume(1.1,1.1));
 assert(!s14::inCollisionVolume(2.4,1.1));
 assert(!s14::inCollisionVolume(NAN,1.1));
 Eigen::Matrix3d narrow=Eigen::Matrix3d::Zero();narrow(0,0)=2.;
 assert(s14::slopeSigma(narrow,.012)>.7);
 Eigen::Matrix3d supported=Eigen::Matrix3d::Identity()*2.;
 assert(s14::slopeSigma(supported,.012)<.7);
 assert(s14::unobservableSlopeRisk(.005,0.,profile)==-1);
 assert(s14::unobservableSlopeRisk(.06,0.,profile)==100);
 assert(s14::unobservableSlopeRisk(-.06,0.,profile)==100);
 assert(s14::unobservableSlopeRisk(.005,5.,profile)==100);
 if(argc>1){
  nlohmann::json fixture;std::ifstream f(argv[1]);f>>fixture;assert(fixture["cases"].size()>=5);
  for(const auto& test:fixture["cases"]){
   std::vector<V> points;V mean=V::Zero();
   for(const auto& p:test["neighbors"]){V v(p[0],p[1],p[2]);points.push_back(v);mean+=v;}
   mean/=points.size();Eigen::Matrix3d sum=Eigen::Matrix3d::Zero();for(auto& p:points)sum+=(p-mean)*(p-mean).transpose();
   assert(test["old_slope"].get<double>()+profile.slope_sigma>profile.slope);
   assert(s14::slopeSigma(sum,profile.height_sigma)>profile.slope_sigma);
   assert(s14::unobservableSlopeRisk(test["height"],fixture["reference_slope"],profile)==-1);
  }
  std::cout<<"ACTUAL_SPARSE_RING_REPLAY_UNKNOWN: "<<fixture["cases"].size()<<" neighborhoods\n";
 }
 std::cout<<"PLANE_INTERCEPT_FIXED; 6cm obstacle and slope veto retained\n";
}
