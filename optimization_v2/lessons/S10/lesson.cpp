// Teaching reduction: an accepted request is not work; measured stop is not OFF command.
#include <cassert>
#include <iostream>
#include <string>
struct Tool {
 std::string phase="IDLE";double stopped_since=-1,last=-1;bool permission=false;
 void request(){phase="ADMITTED";permission=true;}
 bool observe(double t,double measured_rpm){
  if(t<=last||t-last>.1)stopped_since=-1;
  if(measured_rpm>.1)stopped_since=-1;else if(stopped_since<0)stopped_since=t;
  last=t;return stopped_since>=0&&t-stopped_since>=.2;
 }
 void cancel(){permission=false;phase="WAIT_STOP";}
};
int main(){Tool t;t.request();assert(t.phase!="SUCCEEDED");t.cancel();assert(!t.observe(.05,8.));assert(!t.observe(.10,0.));assert(!t.observe(.15,0.));assert(!t.observe(.20,0.));assert(!t.observe(.25,0.));assert(t.observe(.31,0.));assert(!t.permission);std::cout<<"PASS: request != completed; OFF != measured stopped\n";}
