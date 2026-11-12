#pragma once
#include <algorithm>
#include <cmath>

namespace s13 {
struct PlanPoint {double x, y, yaw;};

// Correct only a grid-sized endpoint displacement. Never turn a tolerant
// fallback around a blocked goal into an unchecked exact-goal path.
template<class IsSafe>
bool canUseExactGoal(const PlanPoint & previous, const PlanPoint & snapped,
  const PlanPoint & goal, double resolution, IsSafe is_safe)
{
  for (double v : {previous.x, previous.y, previous.yaw, snapped.x, snapped.y,
      goal.x, goal.y, goal.yaw, resolution}) {
    if (!std::isfinite(v)) {return false;}
  }
  if (resolution <= 0. || std::hypot(goal.x-snapped.x, goal.y-snapped.y) >
    std::sqrt(2.)*resolution+1e-6) {return false;}
  const double distance = std::hypot(goal.x-previous.x, goal.y-previous.y);
  const double dyaw = std::atan2(std::sin(goal.yaw-previous.yaw),
    std::cos(goal.yaw-previous.yaw));
  const int steps = std::max(1, static_cast<int>(std::ceil(std::max(
    distance/(resolution*.5), std::abs(dyaw)/.025))));
  for (int i=0; i<=steps; ++i) {
    const double u = static_cast<double>(i)/steps;
    if (!is_safe(previous.x+u*(goal.x-previous.x),
      previous.y+u*(goal.y-previous.y), previous.yaw+u*dyaw)) {return false;}
  }
  return true;
}
}
