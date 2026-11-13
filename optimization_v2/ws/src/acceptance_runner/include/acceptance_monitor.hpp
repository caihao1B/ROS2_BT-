#pragma once
#include <cmath>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>
namespace s17 {
using json = nlohmann::json;
// Evaluator-only contract: truth stays outside navigation and work credit.
inline json monitor(const json& evidence) {
  if (!evidence.contains("clock") || evidence["clock"].empty())
    throw std::runtime_error("MISSING_CLOCK_EVIDENCE");
  double previous=-1., last=0.; bool monotonic=true;
  for (const auto& item:evidence["clock"]) {
    double t=item.get<double>();
    if (!std::isfinite(t)||t<0.) throw std::runtime_error("INVALID_CLOCK");
    if(t<previous)monotonic=false;
    previous=last=t;
  }
  if (!evidence.contains("contacts") || !evidence["contacts"].is_object())
    throw std::runtime_error("MISSING_CONTACT_EVIDENCE");
  json obstacle=json::object();
  for(auto it=evidence["contacts"].begin();it!=evidence["contacts"].end();++it) {
    const auto name=it.key();
    // Exact named ground surfaces only; no substring exemption for obstacles.
    const bool ground=name.ends_with("|ground::body::ground") ||
                      name.ends_with("|site_terrain::terrain_link::terrain_collision");
    if(!ground && it.value().get<long long>()>0) obstacle[name]=it.value();
  }
  const double limit=evidence.at("limit_s").get<double>();
  const bool stopped=evidence.at("tool_stopped").get<bool>();
  return {{"clock_monotonic",monotonic},{"last_sim_s",last},
    {"within_budget",last<=limit},{"obstacle_contacts",obstacle},
    {"tool_stopped",stopped},
    {"passed",monotonic&&last<=limit&&stopped&&obstacle.empty()}};
}
}
