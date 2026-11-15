import sys, unittest, subprocess, tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import APP, PKG
from global_filter_r10 import derive

class GlobalCorrection(unittest.TestCase):
 def test_original_raw_guards_precede_filter(self):
  original=(APP/'build/r08_source/global_manager.cpp').read_text();derived=derive(original)
  for line in original.splitlines():
   if 'MAP_GNSS_CONFLICT' in line or 'INNOVATION_REJECTED_FIXED' in line or 'if(hold_)' in line:
    self.assertIn(line,derived)
  self.assertIn('if(jump(next)){map_bad_=true;map_t_=-100;map_reason_="MAP_JUMP_REJECTED";',derived)
  self.assertLess(derived.index('if(jump(next))'),derived.index('else correction_=r10::smoothCorrection'))
  self.assertIn('if(allow_&&ready&&!hold_)',derived)
  self.assertIn('else correction_=r10::smoothCorrection',derived)
  self.assertIn('stopped()?c("correction_filter_tau_s"):c("correction_filter_moving_tau_s")',derived)
 def test_cpp_filter_timebase_jitter_and_local_motion(self):
  code='''#include "correction_filter.hpp"
#include <cassert>
int main(){using T=Eigen::Isometry3d;T a=T::Identity(),b=a;b.translation().x()=.1;
assert((r10::smoothCorrection(a,b,0.,.8).matrix()-a.matrix()).norm()==0.);
T one=r10::smoothCorrection(a,b,.8,.8),many=a;
for(int i=0;i<40;++i)many=r10::smoothCorrection(many,b,.02,.8);
assert((one.matrix()-many.matrix()).norm()<1e-12);
assert(std::abs(one.translation().x()-.1*(1.-std::exp(-1.)))<1e-12);
T out=a;double max_error=0.;
for(int i=0;i<500;++i){T noisy=a;noisy.translation().x()=(i%20<10?.025:-.025);out=r10::smoothCorrection(out,noisy,.02,.8);if(i>200)max_error=std::max(max_error,std::abs(out.translation().x()));}
assert(max_error<.005);
T odom=a;odom.translation().x()=2.;assert(std::abs((out*odom).translation().x()-out.translation().x()-2.)<1e-12);
T turn=a;turn.linear()=Eigen::AngleAxisd(.02,Eigen::Vector3d::UnitZ()).toRotationMatrix();
auto r=r10::smoothCorrection(a,turn,.8,.8);assert(std::abs(Eigen::AngleAxisd(r.linear()).angle()-.02*(1.-std::exp(-1.)))<1e-12);
bool rejected=false;try{r10::smoothCorrection(a,b,.1,0.);}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
}'''
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);(p/'check.cpp').write_text(code)
   subprocess.run(['g++','-std=c++17','-O1','-I/usr/include/eigen3','-I'+str(PKG/'r10_backend'),str(p/'check.cpp'),'-o',str(p/'check')],check=True,timeout=20)
   subprocess.run([str(p/'check')],check=True,timeout=5)
