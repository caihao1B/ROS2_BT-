// Production risk function on synthetic teaching input, not a Gazebo result.
#include "risk15.hpp"
#include <fstream>
#include <iostream>
#include <cassert>
int main(int argc,char**argv){
 assert(argc==2);s15::J cfg;std::ifstream(argv[1])>>cfg;
 s15::J robot={{"frame","odom"},{"x",0.},{"y",0.},{"yaw",0.},{"intent_v",.12}};
 s15::J mode={{"probability",1.},{"mean",{{1.,0.},{.8,0.}}},{"covariance",{{{.01,0.},{0.,.01}},{{.02,0.},{0.,.02}}}}};
 s15::J target={{"id",1},{"t",10.},{"dt",.4},{"size",{.5,.5,1.8}},{"algorithm","TEACHING"},{"modes",s15::J::array({mode})}};
 s15::J packet={{"t",10.},{"frame","odom"},{"status","OK"},{"forecasts",s15::J::array({target})}};
 auto risk=s15::assess(packet,robot,10.1,cfg);assert(risk.hold);std::cout<<risk.reason<<" cap="<<risk.cap<<'\n';
 auto old=s15::assess(packet,robot,11.,cfg);assert(!old.hold);std::cout<<old.reason<<": original observation safety remains authoritative\n";
}
