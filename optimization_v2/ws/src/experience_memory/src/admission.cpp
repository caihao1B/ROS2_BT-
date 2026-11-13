#include <initializer_list>
#include "admission.hpp"
extern "C" int memory_admit(double a,double q,double s,double c,double d,double v,double ac,
 int loc,int ver,int exp,int timeout,const double* limits){
 return s16::admit(a,q,s,c,d,v,ac,loc,ver,exp,timeout,limits);
}
