// Production C++ admission function, followed by the unchanged upstream risk function.
#include <cassert>
#include <fstream>
#include <iostream>
#include "admission.hpp"
#include "risk15.hpp"
int main(int argc,char**argv){
 assert(argc==2);double limits[]={.3,.3,.25,2.,2.,2.,.70710678};
 assert(s16::admit(0,0,.05,4,1,.3,0,true,true,false,false,limits)==0);
 assert(s16::admit(0,0,.05,4,-1,.3,0,true,true,false,false,limits)==9);
 assert(s16::admit(0,0,.05,.1,1,.3,0,true,true,false,false,limits)==8);
 assert(s16::admit(0,0,.05,4,1,.3,0,true,true,false,true,limits)==4);
 s15::J cfg;std::ifstream(argv[1])>>cfg;
 s15::J packet={{"t",10.},{"frame","odom"},{"status","OK"},{"cache_hit",true},{"forecasts",s15::J::array()}};
 s15::J target={{"id",1},{"t",10.},{"dt",.4},{"size",{.5,.5,1.6}},{"algorithm","FRESH_IMM"},{"modes",s15::J::array()}};
 target["modes"].push_back({{"probability",1.},{"mean",{{.1,0.},{.2,0.}}},{"covariance",{{{.001,0.},{0.,.001}},{{.001,0.},{0.,.001}}}}});packet["forecasts"].push_back(target);
 s15::J robot={{"frame","odom"},{"x",0.},{"y",0.},{"yaw",0.},{"intent_v",.1},{"intent_w",0.}};
 auto risk=s15::assess(packet,robot,10.,cfg);assert(risk.hold&&risk.cap==0);
 packet["status"]="INFERENCE_TIMEOUT";assert(!s15::assess(packet,robot,10.,cfg).usable);
 std::cout<<"PASS: live matching, opposite direction, conflict and timeout; cache_hit cannot override S15 stop\n";
}
