// Teaching only: transform a local 3D collision envelope into world coordinates.
// Production uses Gazebo Link / pose math in ScriptedMotion.cc, not this executable.
#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
struct Box {std::array<double,3> center,size;};
Box worldBox(Box b,double yaw,std::array<double,3> translation) {
  for(double s:b.size) if(!std::isfinite(s)||s<=0) throw std::invalid_argument("invalid size");
  const double c=cos(yaw),s=sin(yaw);
  return {{{translation[0]+c*b.center[0]-s*b.center[1],translation[1]+s*b.center[0]+c*b.center[1],translation[2]+b.center[2]}},
          {{abs(c)*b.size[0]+abs(s)*b.size[1],abs(s)*b.size[0]+abs(c)*b.size[1],b.size[2]}}};
}
int main() {
  auto w=worldBox({{1,0,.5},{2,1,1}},M_PI/2,{10,20,0});
  if(abs(w.center[0]-10)>1e-9||abs(w.center[1]-21)>1e-9||abs(w.size[0]-1)>1e-9)return 1;
  bool rejected=false;try{worldBox({{0,0,0},{-1,1,1}},0,{0,0,0});}catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)return 2;
  std::cout<<"PASS transformed envelope: center=(10,21,0.5), size=(1,2,1); invalid size rejected\n";
}
