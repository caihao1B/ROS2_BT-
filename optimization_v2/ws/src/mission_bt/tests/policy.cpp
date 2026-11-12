#include "decision.hpp"
#include "tool_permit.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
using J=nlohmann::json;
int main(int argc,char**argv){
 assert(argc==2);J cfg;std::ifstream(argv[1])>>cfg;
 J e={{"t",0.},{"remaining_s",300.},{"stale",false},{"initialized",true},{"blocked",false}};
 s14::Policy p(cfg);auto choose=[&](const char*reason){auto d=p.choose(e);assert(d["reason_code"]==reason);assert(d.contains("candidates")&&d.contains("evidence")&&d.contains("thresholds"));std::cout<<d.dump()<<'\n';};
 choose("NEXT_OBLIGATION");e["blocked"]=true;e["t"]=2.;choose("SHORT_OBSTRUCTION_WAIT");
 e["t"]=6.;e["detour_valid"]=true;e["detour_s"]=2.;choose("LOCAL_DETOUR");
 e["detour_valid"]=false;e["t"]=14.;choose("GLOBAL_REPLAN");
 e["replans"]=2;e["alternative"]=true;choose("DEFER_OBLIGATION");
 e["deferrals"]=3;choose("BLOCK_RETRY_EXHAUSTED");
 e["blocked"]=false;e["t"]=15.;choose("CLEAR_HYSTERESIS_WAIT");e["t"]=16.1;choose("NEXT_OBLIGATION");
 e["residual_only"]=true;e["patch_valid"]=true;e["patch_s"]=15.;choose("RESIDUAL_PATCH");
 e["patches"]=2;choose("PATCH_LIMIT");e["unsafe"]=true;choose("SAFETY_STOP");
 e["unsafe"]=false;e["stale"]=true;choose("INPUT_STALE");e["stale"]=false;e["remaining_s"]=7.;choose("BUDGET_STOP");
 e["remaining_s"]=300.;e["initialized"]=false;e["t"]=90.;choose("STARTUP_ADMISSION_TIMEOUT");
 e["t"]=1.;choose("SENSOR_ADMISSION_WAIT");e["complete"]=true;choose("FEEDBACK_COMPLETE");
 // A missing observation is not evidence that a different route will help.
 e={{"t",40.},{"remaining_s",300.},{"stale",false},{"initialized",true},{"blocked",true},{"admission_recovery",true}};
 choose("OBSERVATION_RECOVERY_WAIT");e["t"]=55.;choose("OBSERVATION_RECOVERY_WAIT");
 e["admission_recovery_exhausted"]=true;choose("BLOCK_RETRY_EXHAUSTED");
 e["unsafe"]=true;choose("SAFETY_STOP");e["unsafe"]=false;e["remaining_s"]=7.;choose("BUDGET_STOP");
 // The production guard keeps the same 8s reserve; the caller must supply
 // remaining motion, not the whole original segment after moving halfway.
 p=s14::Policy(cfg);
 e={{"t",142.},{"remaining_s",158.},{"next_min_s",150.},{"stale",false},{"initialized",true},{"blocked",false},{"busy_action","EXECUTE"}};
 choose("BUDGET_STOP");e["next_min_s"]=90.;choose("NEXT_OBLIGATION");e["remaining_s"]=98.;choose("BUDGET_STOP");
 J tool={{"key","k"},{"active",true},{"working_confirmed",true},{"tool_on_requested",true}},lease={{"work",true},{"tool_key","k"},{"segment","COVERAGE"},{"operation","FOLLOW"}};
 assert(s14::toolPermit(tool,lease));lease["segment"]="TRANSIT";assert(!s14::toolPermit(tool,lease));lease["segment"]="COVERAGE";lease["tool_key"]="other";assert(!s14::toolPermit(tool,lease));lease["tool_key"]="k";tool["working_confirmed"]=false;assert(!s14::toolPermit(tool,lease));
 std::cout<<"POLICY_AND_TOOL_INTERLOCK_PASS\n";
}
