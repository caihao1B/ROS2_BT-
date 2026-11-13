#pragma once
#include <cmath>
#include <initializer_list>
// The caller must still run current predictions through S15 risk + S14 safety.
// Return codes form a stable, tested ABI; no command velocity is produced here.
namespace s16 {
inline int admit(double age,double quality_age,double sigma,double clearance,
                 double direction_cos,double speed,double accel,bool loc_ok,
                 bool version_ok,bool expired,bool timed_out,const double* limits){
 for(double v:{age,quality_age,sigma,clearance,direction_cos,speed,accel})
  if(!std::isfinite(v))return 1;
 if(!version_ok)return 2;
 if(expired)return 3;
 if(timed_out)return 4;
 if(age<0||age>limits[0]||quality_age<0||quality_age>limits[1])return 5;
 if(!loc_ok||sigma<0||sigma>limits[2])return 6;
 if(speed<0||speed>limits[3]||accel<0||accel>limits[4])return 7;
 if(clearance<limits[5])return 8;
 if(direction_cos<limits[6]||direction_cos>1.000001)return 9;
 return 0;
}
}
