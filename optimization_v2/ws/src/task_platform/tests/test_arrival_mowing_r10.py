import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from arrival_mowing_r10 import stable_arrival


class OriginalArrival(unittest.TestCase):
 def window(self,p):return [(i*.03,p) for i in range(23)]
 def test_original_bounds_accept_stopped_arrival(self):
  self.assertTrue(stable_arrival(self.window([.07,0.,.011]),[0.,0.,0.]))
 def test_every_pose_must_meet_original_position_and_heading(self):
  for p in [[.10001,0.,0.],[0.,0.,.12001],[float('nan'),0.,0.]]:
   with self.subTest(p=p):self.assertFalse(stable_arrival(self.window(p),[0.,0.,0.]))
  samples=self.window([0.,0.,0.]);samples[10]=(samples[10][0],[.101,0.,0.]);self.assertFalse(stable_arrival(samples,[0.,0.,0.]))
 def test_single_good_sample_or_stale_gap_does_not_admit(self):
  self.assertFalse(stable_arrival([(0.,[0.,0.,0.])],[0.,0.,0.]))
  s=self.window([0.,0.,0.]);s[12:]=[(t+.2,p) for t,p in s[12:]];self.assertFalse(stable_arrival(s,[0.,0.,0.]))
 def test_angle_wrap_keeps_physical_heading(self):
  self.assertTrue(stable_arrival(self.window([0.,0.,math.pi-.01]),[0.,0.,-math.pi+.01]))
