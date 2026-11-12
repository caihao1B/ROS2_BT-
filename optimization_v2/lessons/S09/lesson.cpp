// The production policy header is exercised; point-cloud extraction stays in terrain_node.
#include "terrain_perception/risk.hpp"
#include <cassert>
#include <iostream>
int main(){
 s09::Profile diff{.04,5.,.035,.22,.30},ack{.06,5.,.025,.14,1.};
 assert(s09::risk(0,0,.005,0,true,diff)==0);
 assert(s09::risk(3,.01,.005,.12,true,diff)<100);
 assert(s09::risk(9,.01,.005,.2,true,diff)==100);
 for(double h:{.08,.15,.30})assert(s09::risk(0,h,.01,h,false,diff)==100);
 assert(s09::risk(2,.035,.01,.03,true,diff)==100);
 assert(s09::risk(2,.035,.01,.03,true,ack)<100);
 assert(s09::risk(2,.01,.03,.03,true,ack)==100);
 assert(s09::risk(2,.01,.03,.03,true,diff)<100);
 int count=0;assert(!s09::clear(100,0,count,3));assert(!s09::clear(100,0,count,3));assert(s09::clear(100,0,count,3));
 assert(s09::risk(0,0,0,NAN,true,diff)==-1);
 std::cout<<"PASS terrain policy / clearing / uncertainty assertions\n";
}
