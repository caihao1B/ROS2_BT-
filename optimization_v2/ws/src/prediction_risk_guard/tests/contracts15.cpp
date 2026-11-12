#include "risk15.hpp"
#include "decision15.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
int main(int argc,char**argv){
 assert(argc==3);nlohmann::json cfg,bt;std::ifstream(argv[1])>>cfg;std::ifstream(argv[2])>>bt;
 using J=nlohmann::json;J robot={{"frame","odom"},{"x",0.},{"y",0.},{"yaw",0.},{"intent_v",.12},{"intent_w",0.}};
 J mode={{"probability",1.},{"mean",{{.5,0.},{1.,0.}}},{"covariance",{{{.01,0.},{0.,.01}},{{.02,0.},{0.,.02}}}}};
 J object={{"id",1},{"t",10.},{"dt",.4},{"size",{.5,.5,1.8}},{"algorithm","test"},{"modes",J::array({mode})}};
 J packet={{"t",10.},{"frame","odom"},{"status","OK"},{"forecasts",J::array({object})}};
 auto r=s15::assess(packet,robot,10.1,cfg);assert(r.hold&&r.cap==0&&r.reason=="PREDICTED_CONFLICT_WAIT");
 assert(!s15::assess(packet,robot,10.7,cfg).hold);
 auto bad=packet;bad["frame"]="map";assert(!s15::assess(bad,robot,10.1,cfg).hold);
 bad=packet;bad["status"]="INFERENCE_TIMEOUT";r=s15::assess(bad,robot,10.1,cfg);assert(!r.hold&&r.cap==.12);
 bad=packet;bad["forecasts"][0]["modes"][0]["covariance"][0]={{100.,0.},{0.,100.}};r=s15::assess(bad,robot,10.1,cfg);assert(!r.hold&&r.reason=="PREDICTION_UNCERTAINTY_FALLBACK");
 bad=packet;bad["forecasts"][0]["modes"][0]["probability"]=-1.;bool rejected=false;try{s15::assess(bad,robot,10.1,cfg);}catch(...){rejected=true;}assert(rejected);
 s15::Policy policy(bt);J e={{"t",10.},{"remaining_s",200.},{"stale",false},{"initialized",true},{"prediction_hold",true}};
 assert(policy.choose(e)["reason_code"]=="PREDICTED_CONFLICT_WAIT");
 e["unsafe"]=true;assert(policy.choose(e)["reason_code"]=="SAFETY_STOP");e["unsafe"]=false;
 e["stale"]=true;assert(policy.choose(e)["reason_code"]=="INPUT_STALE");e["stale"]=false;
 e["remaining_s"]=5.;assert(policy.choose(e)["reason_code"]=="BUDGET_STOP");e["remaining_s"]=200.;
 e["blocked"]=true;assert(policy.choose(e)["reason_code"]=="SHORT_OBSTRUCTION_WAIT");
 std::cout<<"PASS: risk, probability, clock/frame, uncertainty, original safety/budget/geometry priority\n";
}
