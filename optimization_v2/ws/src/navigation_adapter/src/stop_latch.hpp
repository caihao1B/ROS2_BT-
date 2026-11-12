#pragma once
#include <cmath>
// A recovered admission signal cannot cancel braking already in progress.
// Call with fresh measured wheel AND odometry standstill, never command zero.
class StopLatch {
 bool latched_=true; double stable_since_=-1., last_t_=-1.;
public:
 bool update(bool stop_requested, bool measured_stopped, double t) {
  if (!std::isfinite(t)) {latched_=true;stable_since_=-1.;return true;}
  if(t<last_t_) {latched_=true;stable_since_=-1.;}
  last_t_=t;
  if(stop_requested) latched_=true;
  if(!measured_stopped) stable_since_=-1.;
  else if(stable_since_<0.) stable_since_=t;
  if(!stop_requested && stable_since_>=0. && t-stable_since_>=.25) latched_=false;
  return latched_;
 }
};
