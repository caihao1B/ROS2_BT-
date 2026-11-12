#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <string>
namespace s14 {
using J=nlohmann::json;
class Policy {
 J cfg_; double blocked_since_=-1, clear_since_=-1;
 public:
 explicit Policy(J cfg):cfg_(std::move(cfg)){}
 J choose(const J& e) {
  double t=e.value("t",0.),left=e.value("remaining_s",0.);
  bool blocked=e.value("blocked",false);
  if(blocked){if(blocked_since_<0)blocked_since_=t;clear_since_=-1;}
  else{if(clear_since_<0)clear_since_=t;if(t-clear_since_>=cfg_.at("clear_hold_s").get<double>())blocked_since_=-1;}
  double duration=blocked_since_<0?0:t-blocked_since_;
  J candidates=J::array();
  auto candidate=[&](std::string a,double cost,bool feasible,std::string why){candidates.push_back({{"action",a},{"cost_s",cost},{"feasible",feasible},{"reason",why}});};
  candidate("WAIT",cfg_.at("short_wait_s"),left>cfg_.at("stop_reserve_s").get<double>(),"bounded observed-obstacle wait; no prediction");
  candidate("DETOUR",e.value("detour_s",1e6),e.value("detour_valid",false),"requires a collision-checked live path");
  candidate("REPLAN",e.value("replan_s",10.),e.value("replans",0)<cfg_.at("max_replans").get<int>(),"bounded global planner query");
  candidate("DEFER",e.value("defer_s",15.),e.value("alternative",false)&&e.value("deferrals",0)<cfg_.at("max_deferrals").get<int>(),"retains original obligation and denominator");
  candidate("PATCH",e.value("patch_s",1e6),e.value("patch_valid",false)&&e.value("patches",0)<cfg_.at("max_patches").get<int>(),"actual residual geometry only");
  std::string action="EXECUTE",reason="NEXT_OBLIGATION";
  auto set=[&](const char*a,const char*r){action=a;reason=r;};
  if(e.value("stale",true))set("STOP","INPUT_STALE");
  else if(e.value("unsafe",false)||e.value("risk",0.)>=cfg_.at("risk_stop").get<double>())set("STOP","SAFETY_STOP");
  else if(e.value("complete",false))set("COMPLETE","FEEDBACK_COMPLETE");
  else if(left<=cfg_.at("stop_reserve_s").get<double>()+e.value("next_min_s",0.))set("STOP","BUDGET_STOP");
  else if(!e.value("initialized",false)){
   if(t>=cfg_.at("startup_timeout_s").get<double>())set("STOP","STARTUP_ADMISSION_TIMEOUT");
   else set("WAIT","SENSOR_ADMISSION_WAIT");
  }else if(e.value("admission_recovery_exhausted",false))set("STOP","BLOCK_RETRY_EXHAUSTED");
  else if(e.value("admission_recovery",false)){
   if(duration>=cfg_.at("max_block_s").get<double>())set("STOP","BLOCK_RETRY_EXHAUSTED");
   else set("WAIT","OBSERVATION_RECOVERY_WAIT");
  }
  else if(e.value("busy_action",std::string())=="REPLAN"&&!blocked)set("REPLAN","GLOBAL_REPLAN");
  else if(!blocked&&blocked_since_>=0)set("WAIT","CLEAR_HYSTERESIS_WAIT");
  else if(blocked){
   if(duration>=cfg_.at("max_block_s").get<double>())set("STOP","BLOCK_RETRY_EXHAUSTED");
   else if(duration<cfg_.at("short_wait_s").get<double>())set("WAIT","SHORT_OBSTRUCTION_WAIT");
   else if(e.value("detour_valid",false)&&e.value("detour_s",1e6)<cfg_.at("persistent_block_s").get<double>()-duration&&left>e.value("detour_s",1e6)+cfg_.at("stop_reserve_s").get<double>())set("DETOUR","LOCAL_DETOUR");
   else if(duration>=cfg_.at("persistent_block_s").get<double>()){
    if(e.value("replans",0)<cfg_.at("max_replans").get<int>()&&left>e.value("replan_s",10.)+cfg_.at("stop_reserve_s").get<double>())set("REPLAN","GLOBAL_REPLAN");
    else if(e.value("alternative",false)&&e.value("deferrals",0)<cfg_.at("max_deferrals").get<int>())set("DEFER","DEFER_OBLIGATION");
    else set("STOP","BLOCK_RETRY_EXHAUSTED");
   }else set("WAIT","SHORT_OBSTRUCTION_WAIT");
  }else if(e.value("residual_only",false)){
   if(e.value("patch_valid",false)&&e.value("patches",0)<cfg_.at("max_patches").get<int>()&&left>e.value("patch_s",1e6)+cfg_.at("stop_reserve_s").get<double>())set("PATCH","RESIDUAL_PATCH");
   else set("STOP","PATCH_LIMIT");
  }
  for(auto& c:candidates)c["selected"]=c["action"]==action;
  return {{"t",t},{"action",action},{"reason_code",reason},{"evidence",e},{"blocked_duration_s",duration},{"thresholds",cfg_},{"candidates",candidates},{"prediction_source","CURRENT_OBSERVATIONS_ONLY"}};
 }
};
}
