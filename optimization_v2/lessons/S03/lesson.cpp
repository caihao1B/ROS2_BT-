// Teaching only: bounded joint mapping; real leased actuator is WheelDrive.cc.
#include <cmath>
#include <iostream>
#include <stdexcept>
struct Targets{double left,right,steerLeft,steerRight;};
Targets ackermann(double v,double w){
  if(!std::isfinite(v)||!std::isfinite(w))throw std::invalid_argument("nonfinite request");
  if(std::abs(v)<.001)return {0,0,0,0};
  double k=std::fmax(-.5,std::fmin(.5,w/v));
  return {v*(1-.60*k/2)/.18,v*(1+.60*k/2)/.18,std::atan(k/(1-.70*k/2)),std::atan(k/(1+.70*k/2))};
}
int main(){auto a=ackermann(.6,.3);auto z=ackermann(0,1);
  if(a.left>=a.right||a.steerLeft<=a.steerRight||z.left!=0||z.right!=0)return 1;
  bool rejected=false;try{ackermann(NAN,0);}catch(const std::invalid_argument&){rejected=true;}
  if(!rejected)return 2;
  std::cout<<"PASS: steering="<<a.steerLeft<<","<<a.steerRight<<" rad; zero-speed spin blocked; NaN rejected\n";
}
