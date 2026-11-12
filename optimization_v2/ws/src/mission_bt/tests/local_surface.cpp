#include <cassert>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "local_surface.hpp"
using V=Eigen::Vector3d;
int main(int argc,char** argv){
 s09::Profile profile{.04,5.,.035,.22,.3};
 auto cost=[&](const std::vector<V>& points,V center,double height,double span=0.){
  auto f=s14::localSurface(points,center,.025,.012);
  if(!f.valid || f.sigma>.7)return s14::unobservableSlopeRisk(height,0.,profile);
  double step=std::max(f.step,span);
  if(height<-.045)step=std::max(step,std::abs(height));
  return s09::risk(f.slope,step,f.rough,height,f.rough<.02&&f.slope<15.,profile);
 };
 std::vector<V> flat;
 for(int y=-3;y<=3;++y)for(int x=-3;x<=3;++x)flat.emplace_back(x*.15,y*.15,0.);
 assert(cost(flat,V::Zero(),0.)==0);
 auto neighbor=flat;neighbor.back().z()=.18;
 assert(cost(neighbor,V::Zero(),0.)==0); // distant point does not occupy this cell
 assert(cost(flat,V(0,0,.06),.06)==100); // positive step at the cell
 assert(cost(flat,V(0,0,-.06),-.06)==100); // negative step at the cell
 assert(cost(flat,V::Zero(),0.,.06)==100); // mixed ground/rock within the cell
 auto island=flat;for(auto& v:island)v.z()=.06;
 assert(cost(island,V(0,0,.06),.06)==100); // flat raised top is not new ground
 auto pit=flat;for(auto& v:pit)v.z()=-.06;
 assert(cost(pit,V(0,0,-.06),-.06)==100);
 auto slope=flat;for(auto& v:slope)v.z()=v.x()*std::tan(5.*M_PI/180);
 assert(cost(slope,V::Zero(),0.)==100);
 std::vector<V> ring;for(int x=-5;x<=5;++x)ring.emplace_back(.15*x,0.,0.);
 assert(cost(ring,V::Zero(),0.)==-1);
 assert(cost(ring,V::Zero(),.06)==100);
 assert(cost({},V::Zero(),0.)==-1);
 if(argc>1){
  nlohmann::json fixture;std::ifstream input(argv[1]);input>>fixture;
  assert(fixture["cases"].size()==26);
  for(const auto& c:fixture["cases"]){
   std::vector<V> neighbors;for(const auto& v:c["neighbors"])neighbors.emplace_back(v[0],v[1],v[2]);
   const auto& v=c["center"];V center(v[0],v[1],v[2]);
   assert(c["old_step"].get<double>()>.06);
   auto fit=s14::localSurface(neighbors,center,.025,.012);
   assert(fit.valid && fit.step<.009 && fit.sigma<.7);
   assert(cost(neighbors,center,c["top"])==0);
  }
  std::cout<<"ACTUAL_TREE_NEIGHBOR_REPLAY_PASS: "<<fixture["cases"].size()<<" cells\n";
 }
 std::cout<<"POSITIVE_NEGATIVE_6CM_AND_5DEG_VETO_RETAINED; sparse support UNKNOWN\n";
}
