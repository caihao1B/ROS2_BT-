#include "self_filter.hpp"
#include <cassert>
#include <iostream>
int main(){std::array<double,4>b={-.621,2.259,-1.435,1.435};
assert(r09::selfReturn(0,0,.4,b,.05,1.188));
assert(r09::selfReturn(-.6,0,1.13,b,.05,1.188));
assert(!r09::selfReturn(-.6,0,0.,b,.05,1.188));
assert(!r09::selfReturn(-.6,0,.04,b,.05,1.188));
assert(!r09::selfReturn(-.6,0,-.1,b,.05,1.188));
assert(!r09::selfReturn(0,1.42,.04,b,.05,1.188));
assert(!r09::selfReturn(0,1.5,.4,b,.05,1.188));
assert(!r09::selfReturn(0,0,1.4,b,.05,1.188));
std::cout<<"8 self-filter checks passed; observed ground/low steps retained\n";}
