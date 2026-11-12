// Teaching example; production uses the actual Nav2 plugins and command_gate.cpp.
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include "../../ws/src/navigation_adapter/src/stop_latch.hpp"
#include "../../ws/src/navigation_adapter/src/self_mask.hpp"
#include "../../ws/src/navigation_adapter/src/observation_confirmation.hpp"
#include "../../ws/src/navigation_adapter/src/exact_goal.hpp"
struct Command{double v,w;};
Command constrain(Command u,bool ack,bool admitted){
 if(!admitted||!std::isfinite(u.v)||!std::isfinite(u.w))return {0,0};
 u.v=std::clamp(u.v,0.,.12);u.w=std::clamp(u.w,-.35,.35);
 if(ack)u.w=std::clamp(u.w,-u.v/2.11,u.v/2.11);
 return u;
}
int main(){
 const s13::PlanPoint before{0.,0.,0.}, snapped{.1,0.,0.}, exact{.12,.01,.1};
 assert(s13::canUseExactGoal(before,snapped,exact,.05,[](double,double,double){return true;}));
 assert(!s13::canUseExactGoal(before,snapped,exact,.05,[](double x,double,double){return x<.06;}));
 assert(!s13::canUseExactGoal(before,snapped,{.3,0.,0.},.05,[](double,double,double){return true;}));
 assert(!s13::canUseExactGoal(before,snapped,exact,0.,[](double,double,double){return true;}));
 int count=0;count=s13::confirmLethal(count,true,1.);assert(count==1);
 count=s13::confirmLethal(count,true,.1);assert(count==2);
 count=s13::confirmLethal(count,true,.1);assert(count==3);
 assert(s13::confirmLethal(count,true,.4)==1);
 assert(s13::confirmLethal(count,false,.1)==0);
 const std::array<double,4> diff_mask={-.55,.60,-.45,.45};
 assert(s13::selfPoint(.3,0,.3,diff_mask));
 assert(!s13::selfPoint(1.1,0,.08,diff_mask)); // Keep the actual low obstacle.
 assert(!s13::selfPoint(.3,.7,0,diff_mask)); // Keep visible surrounding ground.
 StopLatch brake;
 assert(brake.update(true,false,1.));
 assert(brake.update(false,false,1.01)); // Admission recovered, still moving.
 assert(brake.update(false,true,1.1));
 assert(brake.update(false,true,1.3)); // Not stable for 0.25 s yet.
 assert(!brake.update(false,true,1.36));
 assert(brake.update(true,false,2.));
 assert(brake.update(false,true,2.1));
 assert(brake.update(false,false,2.3)); // Feedback loss resets stability.
 assert(brake.update(false,true,2.4));
 assert(!brake.update(false,true,2.66));
 assert(brake.update(false,true,.5)); // Clock rewind latches again.
 auto u=constrain({.3,.2},true,true);assert(u.v==.12&&std::abs(u.w-.12/2.11)<1e-9);
 auto stopped=constrain({0,.2},true,true);assert(stopped.w==0);
 auto denied=constrain({.12,.1},false,false);assert(denied.v==0&&denied.w==0);
 auto invalid=constrain({NAN,.1},false,true);assert(invalid.v==0&&invalid.w==0);
 double k=u.w/u.v,left=std::atan(k/(1-.35*k)),right=std::atan(k/(1+.35*k));
 std::cout<<"PASS: v="<<u.v<<" w="<<u.w<<" left="<<left<<" right="<<right<<"; limits, stable stop latch, self mask and fresh terrain confirmation verified\n";
}
