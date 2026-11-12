#include <cassert>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include "risk_evidence.hpp"
#include "slope_observability.hpp"

int main(int argc, char** argv) {
  using J = nlohmann::json;
  assert(argc == 2);
  J fixture; std::ifstream in(argv[1]); in >> fixture;
  const double ttl = fixture["unknown_after_s"];
  const int clear_frames = fixture["clear_frames"];
  assert(ttl == 2. && clear_frames == 3);
  s09::Profile profile{.04, 5., .035, .22, .3};
  int stored = 100, count = 0, expired = 0;
  // The true last strong observation is no later than the first recorded
  // UNKNOWN sample. This upper bound is conservative for the aging replay.
  double last = fixture["samples"][0]["t"];
  const double first = last;
  for (const auto& s : fixture["samples"]) {
    const auto& c = s["cell"];
    assert(c[8] == 100 && !c[11].get<bool>());
    assert(c[10].get<double>() > profile.slope_sigma);
    double slope = std::acos(std::clamp(s["ground_plane"][2].get<double>(), 0., 1.))*180/M_PI;
    const int candidate = s14::unobservableSlopeRisk(c[3], slope, profile);
    assert(candidate == -1);
    const double t = s["t"];
    s14::observeRisk(candidate, t, clear_frames, stored, count, last);
    assert(last == first && stored == 100 && count == 0);
    const int active = s14::riskAt(stored, last, t, ttl);
    assert(active != 0);
    if (t-first > ttl) {assert(active == -1); ++expired;}
    else assert(active == 100);
  }
  assert(expired >= 2);
  // Fresh direct hazards continue to stop, including weak local slope support.
  for (double height : {.06, -.06}) {
    for (int i=0; i<100; ++i) {
      const double t = 300.+i*.1;
      const int candidate = s14::unobservableSlopeRisk(height, 0., profile);
      assert(candidate == 100);
      s14::observeRisk(candidate, t, clear_frames, stored, count, last);
      assert(s14::riskAt(stored, last, t, ttl) == 100);
    }
  }
  assert(s14::unobservableSlopeRisk(.005, 5., profile) == 100);
  // Two free candidates do not clear a lethal value or refresh its timestamp.
  stored=100;count=0;last=500.;
  for (int i=1;i<=2;++i) {
    s14::observeRisk(0,500.+i*.1,3,stored,count,last);
    assert(stored==100 && last==500.);
  }
  // UNKNOWN interrupts the required consecutive-clear sequence.
  s14::observeRisk(-1,500.3,3,stored,count,last);
  assert(stored==100 && count==0 && last==500.);
  for (int i=1;i<=3;++i) {
    s14::observeRisk(0,500.3+i*.1,3,stored,count,last);
    assert(stored == (i<3 ? 100 : 0));
  }
  assert(s14::riskAt(100,600.,602.,ttl)==100);
  assert(s14::riskAt(100,600.,602.01,ttl)==-1);
  assert(s14::riskAt(0,600.,599.,ttl)==-1);
  assert(s14::riskAt(100,NAN,600.,ttl)==-1);
  std::cout << "ACTUAL_STALE_RISK_REPLAY_PASS: " << fixture["samples"].size()
            << " samples; 2s age, 3-frame clearing, positive/negative obstacles retained\n";
}
