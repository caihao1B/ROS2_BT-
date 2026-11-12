#pragma once
#include "decision.hpp"
namespace s15 {
class Policy {
 s14::Policy base_;
 public:
 explicit Policy(nlohmann::json cfg):base_(cfg){}
 nlohmann::json choose(const nlohmann::json& evidence){
  auto d=base_.choose(evidence);
  // Observed obstacles, stale inputs, budget and tool faults retain S14 priority.
  if(d["action"]=="EXECUTE"){
   if(evidence.value("prediction_exhausted",false)){d["action"]="STOP";d["reason_code"]="PREDICTION_WAIT_EXHAUSTED";}
   else if(evidence.value("prediction_hold",false)||evidence.value("prediction_recovery",false)){
    d["action"]="WAIT";d["reason_code"]="PREDICTED_CONFLICT_WAIT";
   }
  }
  d["prediction_source"]=evidence.value("prediction_source",std::string("BASELINE_FALLBACK"));
  d["prediction_reason"]=evidence.value("prediction_reason",std::string("NOT_READY"));
  for(auto& c:d["candidates"]){c["selected"]=c["action"]==d["action"];if(c["action"]=="WAIT")c["reason"]="bounded measured/predicted conflict; independent current-obstacle priority";}
  return d;
 }
};
}
