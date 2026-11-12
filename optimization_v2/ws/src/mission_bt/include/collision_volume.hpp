#pragma once
#include <cmath>
namespace s14 {
// A rejected overhead return supplies no evidence that ground is traversable.
inline bool inCollisionVolume(double height, double ceiling) {
 return std::isfinite(height) && std::isfinite(ceiling) && ceiling>0 && height<=ceiling;
}
}
