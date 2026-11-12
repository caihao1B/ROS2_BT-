#pragma once
#include <nlohmann/json.hpp>
#include <deque>
#include <cmath>
namespace s14 {
class ContextBuffer {
  struct Sample {nlohmann::json value;double wall;};
  std::deque<Sample> pending_;
  double last_clock_=-1.;bool rewind_=false;
public:
  void clear(){pending_.clear();}
  void push(nlohmann::json value,double wall) {
    const double t=value.value("t",-1.);
    if(!std::isfinite(t)||t<0){clear();return;}
    pending_.push_back({std::move(value),wall});
    while(pending_.size()>32)pending_.pop_front();
  }
  nlohmann::json select(double now,double wall,double max_age,double max_wall) {
    if(now<last_clock_-.001)rewind_=true;
    last_clock_=now;
    const Sample * selected=nullptr;
    // /clock and context are separate DDS streams. Queue a context delivered
    // before its clock tick; do not treat it as current or extend any timeout.
    for(const auto & s:pending_) {
      const double t=s.value.at("t");
      if(t<=now&&(!selected||t>=selected->value.at("t").get<double>()))selected=&s;
    }
    auto e=selected?selected->value:nlohmann::json::object();
    const double age=now-e.value("t",-100.);
    e["context_source_t"]=e.value("t",-100.);
    e["context_age_s"]=age;
    e["context_wall_age_s"]=selected?wall-selected->wall:1e9;
    e["stale"]=rewind_||!selected||age<-.001||age>max_age||wall-selected->wall>max_wall;
    return e;
  }
};
}
