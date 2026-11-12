#pragma once
#include <cmath>
#include <terrain_perception/risk.hpp>

namespace s14 {
// A new cloud is not necessarily a new, supported risk classification. Keep
// the timestamp of the accepted class, not the timestamp of arbitrary returns.
inline void observeRisk(int candidate, double t, int required_clear_frames,
                        int& stored, int& clear_count, double& evidence_t) {
  if (candidate < 0) {
    clear_count = 0;
    if (stored != 100) stored = -1;
    return;
  }
  if (s09::clear(stored, candidate, clear_count, required_clear_frames)) {
    stored = candidate;
    evidence_t = t;
  }
}

// Expired evidence is UNKNOWN, never free. Repeatedly confirmed obstacle
// memory is maintained separately by the unchanged S13 prior fusion node.
inline int riskAt(int stored, double evidence_t, double now, double max_age) {
  const double age = now - evidence_t;
  if (!std::isfinite(age) || age < 0 || age > max_age) return -1;
  return stored;
}
}
