// Numerical Reeds-Shepp primitives from the installed OMPL implementation.
// Collision checking is deliberately separate and uses the full vehicle body.
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <cmath>
using json=nlohmann::json;
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("request.json result.json required");
 std::ifstream file(argv[1]);json j;file>>j;double r=j.at("radius");if(!std::isfinite(r)||r<2.998)throw std::runtime_error("turn radius contract");
 ompl::base::ReedsSheppStateSpace space(r);auto *a=space.allocState(),*b=space.allocState();json result=json::array();
 auto set=[](ompl::base::State* s,const json& p){auto *q=s->as<ompl::base::SE2StateSpace::StateType>();q->setX(p.at(0));q->setY(p.at(1));q->setYaw(p.at(2));};
 for(const auto& q:j.at("queries")){set(a,q.at("start"));set(b,q.at("goal"));auto path=space.reedsShepp(a,b);json parts=json::array();
  for(int i=0;i<5;++i){double len=path.length_[i]*r;if(std::abs(len)<1e-9)continue;int type=path.type_[i];double k=type==1?1/r:type==3?-1/r:0;parts.push_back({k,len});}
  result.push_back({{"id",q.at("id")},{"primitives",parts},{"length_m",path.length()*r}});
 }
 space.freeState(a);space.freeState(b);std::ofstream out(argv[2]);out<<json({{"solver","OMPL ReedsSheppStateSpace"},{"paths",result}}).dump();return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
