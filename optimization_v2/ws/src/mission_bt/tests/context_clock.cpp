#include "context_clock.hpp"
#include <cassert>
#include <iostream>
int main(){
  s14::ContextBuffer q;
  q.push({{"t",7.12},{"idle",true}},100.);
  q.push({{"t",7.13},{"idle",false}},100.01);
  auto a=q.select(7.12,100.02,.3,.5);
  assert(!a.at("stale").get<bool>()&&a.at("idle").get<bool>());
  auto b=q.select(7.13,100.03,.3,.5);
  assert(!b.at("stale").get<bool>()&&!b.at("idle").get<bool>());
  assert(q.select(7.44,100.04,.3,.5).at("stale").get<bool>());
  s14::ContextBuffer w;w.push({{"t",10.}},1.);
  assert(w.select(10.,1.51,.3,.5).at("stale").get<bool>());
  s14::ContextBuffer r;r.push({{"t",10.}},1.);r.select(10.,1.,.3,.5);
  r.push({{"t",9.}},1.1);assert(r.select(9.,1.1,.3,.5).at("stale").get<bool>());
  std::cout<<"DDS_CLOCK_CONTEXT_REORDERED; AGE_WALL_REWIND_GUARDS_PRESERVED\n";
}
