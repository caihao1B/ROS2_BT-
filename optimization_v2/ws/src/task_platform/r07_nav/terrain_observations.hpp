#pragma once
#include <array>
#include <deque>
#include <algorithm>
#include <vector>
#include <cmath>
namespace r07 {
struct GroundObservation { bool qualified=false; double ground=0,top=0; };
// Three distinct current ray frames, never an assumed free-ground value.
// Strong protrusions/depressions retain immediate protection. Small ground
// variations need consistent observations before they can authorize traversal.
inline GroundObservation groundObservation(std::deque<std::array<double,3>>& history,
 double t,double ground,double top,double obstacle_height) {
 if(!history.empty() && (t<=history.back()[0] || t-history.back()[0]>.25))history.clear();
 history.push_back({t,ground,top});
 while(!history.empty() && (history.size()>3 || t-history.front()[0]>.4))history.pop_front();
 if(top>=obstacle_height || top<=-obstacle_height)return {true,ground,top};
 if(history.size()<3)return {};
 std::vector<double> g,z;for(const auto& row:history){g.push_back(row[1]);z.push_back(row[2]);}
 std::sort(g.begin(),g.end());std::sort(z.begin(),z.end());
 return {true,g[1],z[1]};
}
}
