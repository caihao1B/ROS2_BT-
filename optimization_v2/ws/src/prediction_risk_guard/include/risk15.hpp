#pragma once
#include <nlohmann/json.hpp>
#include <cmath>
#include <algorithm>
#include <string>
namespace s15 {
using J=nlohmann::json;
struct Risk { bool usable=false,hold=false; double mass=0,cap=.12; std::string reason="PREDICTION_BASELINE_FALLBACK";J conflicts=J::array(); };
inline double sigma(const J& p){
 if(!p.is_array()||p.size()!=2||p[0].size()!=2||p[1].size()!=2)throw std::runtime_error("covariance shape");
 double a=p[0][0],b=(p[0][1].get<double>()+p[1][0].get<double>())/2.,d=p[1][1];
 double disc=std::hypot(a-d,2*b),lo=(a+d-disc)/2.,hi=(a+d+disc)/2.;
 if(!std::isfinite(hi)||lo< -1e-6)throw std::runtime_error("non PSD covariance");
 return std::sqrt(std::max(hi,0.));
}
inline Risk assess(const J& packet,const J& robot,double now,const J& cfg){
 Risk r;
 if(packet.value("frame",std::string())!="odom"||robot.value("frame",std::string())!="odom"){r.reason="PREDICTION_FRAME_FALLBACK";return r;}
 if(packet.value("status",std::string())!="OK"){r.reason="PREDICTION_"+packet.value("status",std::string("MISSING"))+"_FALLBACK";return r;}
 double age=now-packet.value("t",-100.);
 if(age<-.05||age>cfg.at("forecast_max_age_sim_s").get<double>()){r.reason="PREDICTION_STALE_FALLBACK";return r;}
 auto c=cfg.at("risk");r.usable=true;r.reason="PREDICTED_CLEAR";bool uncertain=false;
 for(const char* key:{"x","y","yaw"})if(!std::isfinite(robot.at(key).get<double>()))throw std::runtime_error("nonfinite robot state");
 if(!std::isfinite(robot.value("intent_v",0.))||!std::isfinite(robot.value("intent_w",0.)))throw std::runtime_error("nonfinite robot intent");
 double v=std::clamp(robot.value("intent_v",0.),0.,.12),w=std::clamp(robot.value("intent_w",0.),-.35,.35);
 for(const auto& object:packet.at("forecasts")){
  const double dt=object.at("dt"),object_age=now-object.at("t").get<double>();
  if(!std::isfinite(dt)||dt<=0||dt>1.)throw std::runtime_error("invalid prediction interval");
  if(object_age<-.05||object_age>.6)continue;
  double probability_sum=0,mass=0;bool invalid=false;
  for(const auto& mode:object.at("modes")){
   const double probability=mode.at("probability");probability_sum+=probability;
   if(!std::isfinite(probability)||probability<0)throw std::runtime_error("invalid probability");
   const auto& means=mode.at("mean");const auto& cov=mode.at("covariance");
   if(means.size()!=cov.size()||means.empty())throw std::runtime_error("invalid horizon");
   bool intersects=false;
   for(size_t i=0;i<means.size();++i){
    if(dt*(i+1)>c.at("risk_horizon_s").get<double>()+1e-8)break;
    double sd=sigma(cov[i]);
    if(sd>c.at("max_prediction_sigma_m").get<double>()&&probability>.01){invalid=true;break;}
    const double tau=dt*(i+1)-object_age;if(tau<0)continue;
    double heading=robot.at("yaw"),x=robot.at("x"),y=robot.at("y");
    if(std::abs(w)<1e-5){x+=v*tau*std::cos(heading);y+=v*tau*std::sin(heading);}
    else{x+=v/w*(std::sin(heading+w*tau)-std::sin(heading));y-=v/w*(std::cos(heading+w*tau)-std::cos(heading));}
    auto size=object.at("size");double target_radius=.5*std::hypot(size[0].get<double>(),size[1].get<double>());
    // Time discretization allowance for <=2 m/s targets, independent of their mean.
    double radius=c.at("robot_radius_m").get<double>()+target_radius+c.at("clearance_m").get<double>()+c.at("confidence_sigma").get<double>()*sd+.5*dt*(2.+v);
    double ox=means[i][0],oy=means[i][1];if(!std::isfinite(ox)||!std::isfinite(oy))throw std::runtime_error("nonfinite mean");
    if(std::hypot(x-ox,y-oy)<=radius)intersects=true;
   }
   if(intersects)mass+=probability;
  }
  if(std::abs(probability_sum-1.)>.001)throw std::runtime_error("probability normalization");
  if(invalid){uncertain=true;continue;}
  if(mass>0)r.conflicts.push_back({{"id",object.at("id")},{"mode_envelope_mass",mass},{"algorithm",object.at("algorithm")}});
  r.mass=std::max(r.mass,mass);
 }
 // This is a conservative envelope mass proxy, not a calibrated collision probability.
 if(r.mass>=c.at("stop_envelope_mass").get<double>()){r.hold=true;r.cap=0;r.reason="PREDICTED_CONFLICT_WAIT";}
 else if(r.mass>=c.at("slow_envelope_mass").get<double>()){r.cap=c.at("slow_cap_mps");r.reason="PREDICTED_CONFLICT_SLOW";}
 else if(uncertain){r.reason="PREDICTION_UNCERTAINTY_FALLBACK";}
 return r;
}
}
