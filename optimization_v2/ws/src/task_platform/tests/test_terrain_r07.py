"""Compile and exercise the same observation filter included by production terrain."""
import unittest,tempfile,subprocess
from pathlib import Path
class GroundHistory(unittest.TestCase):
 def test_noise_hazards_and_missing_observations(self):
  header=Path(__file__).resolve().parents[1]/'r07_nav/terrain_observations.hpp'
  code='''#include "HEADER"
#include <cassert>
int main(){
 std::deque<std::array<double,3>> h;
 auto feed=[&](double t,double z){return r07::groundObservation(h,t,z,z,.045);};
 assert(!feed(1.,0).qualified);assert(!feed(1.1,.031).qualified);
 auto q=feed(1.2,.003);assert(q.qualified && q.top<.005);
 q=feed(1.3,.08);assert(q.qualified && q.top==.08); // immediate low solid obstacle
 q=feed(1.4,-.08);assert(q.qualified && q.top==-.08); // immediate depression
 q=feed(2.,0);assert(!q.qualified); // stale history never authorizes ground
 assert(!feed(2.1,.04).qualified);q=feed(2.2,.04);assert(q.qualified && q.top==.04);
 q=feed(2.1,0);assert(!q.qualified); // backward epoch discards old evidence
 h.clear();feed(3.,.001);feed(3.1,.002);q=feed(3.2,.001);assert(q.qualified && q.top==.001);
}'''.replace('HEADER',str(header))
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);(p/'test.cpp').write_text(code);subprocess.run(['c++','-std=c++17','-O0',str(p/'test.cpp'),'-o',str(p/'test')],check=True,timeout=15);subprocess.run([str(p/'test')],check=True,timeout=5)
 def test_larger_observed_baseline_resolves_body_shadow_without_relaxing_sigma(self):
  base=Path(__file__).resolve().parents[5]/'optimization_v2/ws/src'
  header=base/'mission_bt/include/local_surface.hpp'
  code='\n'.join(['#include "'+str(header)+'"','#include <cassert>',
   'int main(){auto fit=[](int r){std::vector<Eigen::Vector3d> p;for(int y=-r;y<=r;++y)for(int x=-r;x<=r;++x){double xx=x*.15,yy=.432+y*.15;if(std::abs(xx)<.545 && std::abs(yy)<.385)continue;p.emplace_back(xx,yy,0.);}return s14::localSurface(p,{0,.432,0},.025,.012);}; auto small=fit(3),large=fit(4);assert(small.valid && large.valid);assert(small.sigma>.7 && large.sigma<.7);assert(large.slope<1e-6 && large.step<1e-6);}'])
  with tempfile.TemporaryDirectory() as tmp:
   p=Path(tmp);(p/'plane.cpp').write_text(code);subprocess.run(['c++','-std=c++17','-O0','-I/usr/include/eigen3','-I'+str(base/'terrain_perception/include'),str(p/'plane.cpp'),'-o',str(p/'plane')],check=True,timeout=15);subprocess.run([str(p/'plane')],check=True,timeout=5)
if __name__=='__main__':unittest.main()
