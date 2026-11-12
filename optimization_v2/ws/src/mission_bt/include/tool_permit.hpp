#pragma once
#include <nlohmann/json.hpp>
namespace s14 {
inline bool toolPermit(const nlohmann::json& tool,const nlohmann::json& lease){
 if(!tool.is_object()||!lease.is_object())return false;
 if(lease.value("work",false))return lease.value("segment",std::string())=="COVERAGE"&&lease.value("operation",std::string())=="FOLLOW"&&!lease.value("tool_key",std::string()).empty()&&tool.value("key",std::string())==lease.value("tool_key",std::string())&&tool.value("active",false)&&tool.value("working_confirmed",false)&&tool.value("tool_on_requested",false);
 return tool.value("stopped_confirmed",false)&&!tool.value("tool_on_requested",true)&&!tool.value("active",true);
}
}
