#pragma once
#include <algorithm>
namespace s13 {
// Confirmation controls only additional S13 long-term memory. Every raw lethal
// candidate still reaches the unchanged S09 protective stop and live costmap.
inline int confirmLethal(int previous, bool lethal, double observation_gap) {
 if(!lethal)return 0;
 return std::min(3,(observation_gap>.3?0:previous)+1);
}
}
