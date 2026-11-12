#pragma once
#include <array>
namespace s13 {
// Bounds come from this vehicle's declared footprint plus 5 cm uncertainty.
// This removes self returns only; surrounding observed ground is retained.
inline bool selfPoint(double x,double y,double z,const std::array<double,4>& b){
 return x>b[0]&&x<b[1]&&y>b[2]&&y<b[3]&&z<1.1;
}
}
