#include "policy.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
using r06::J;
int main(int argc,char**argv){
 assert(argc==3);J cfg;std::ifstream(argv[1])>>cfg;cfg["precision_progress_policy"]={{"scope","validated local docking"}};
 J e={{"stale",false},{"bindings_valid",true},{"sensors_healthy",true},{"remaining_s",600.},{"stopped",true},{"tool_stopped",true},{"nav_idle",true},{"localized",true},{"commanded_motion",true},{"blocked",false},{"progress_m",0.},{"motion_scope","VALIDATED_LOCAL_DOCKING"},{"expected_translation_m",.0367}};
 assert(std::abs(r10::progressDelta(cfg,e)-.009175)<1e-9);
 // Small measured advances of the actual few-centimetre motion remain progress.
 r06::Policy fine(cfg);BT::BehaviorTreeFactory factory;fine.register_nodes(factory);auto tree=factory.createTreeFromFile(argv[2]);fine.prepare(e,1.);tree.tickOnce();assert(fine.d["action"]=="EXECUTE");
 for(int i=1;i<=6;++i){e["progress_m"]=i*.006;fine.prepare(e,1.+i*2.);tree.tickOnce();assert(!fine.manual);assert(fine.d["action"]=="EXECUTE");}
 // A real stall still reaches the unchanged timeout and latches manual handling.
 fine.prepare(e,25.);tree.tickOnce();assert(fine.manual);assert(fine.d["action"]=="HUMAN");assert(fine.manual_reason=="MOTION_WITHOUT_OBSERVED_PROGRESS");
 // Ordinary motion retains the original 10 cm requirement.
 e.erase("motion_scope");e["progress_m"]=0.;r06::Policy coarse(cfg);coarse.prepare(e,1.);e["progress_m"]=.0367;coarse.prepare(e,12.);assert(coarse.manual);
 // Oversized, missing or nonfinite requests cannot choose the fine threshold.
 e["motion_scope"]="VALIDATED_LOCAL_DOCKING";e["expected_translation_m"]=.151;assert(r10::progressDelta(cfg,e)==cfg["progress_delta_m"]);
 e["expected_translation_m"]=nullptr;assert(r10::progressDelta(cfg,e)==cfg["progress_delta_m"]);
 std::cout<<J({{"passed",true},{"checks",5},{"coarse_delta_m",cfg["progress_delta_m"]},{"timeout_s",cfg["progress_timeout_s"]},{"scope","Actual derived BT policy; no disabled stall watchdog"}}).dump()<<'\n';
}
