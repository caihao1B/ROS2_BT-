import unittest,sys,json,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from motion_consistency_r08 import assess
class IndependentMotion(unittest.TestCase):
 def fixture(self,icp,wheel,gnss=.2):
  ts=np.linspace(0,.8,9);history=[(t,np.array([icp*t,0.,0.]),np.array([0.,0.,0.,1.]),np.full(6,.001)) for t in ts];fixes=[dict(t=t,valid=True,mode='fixed',antenna_base_m=[-.1,0,.58],antenna=[gnss*t-.1,0,.58],covariance=np.diag([.0009,.0009,.0021]).ravel().tolist()) for t in ts[::2]];wheels=[(t,wheel) for t in ts];return history,fixes,wheels
 def test_false_lidar_motion_does_not_label_real_wheels_slipping(self):
  d=assess(*self.fixture(.9,.2),.8);self.assertFalse(d['icp_valid']);self.assertTrue(d['wheel_valid']);json.dumps(d)
 def test_true_wheel_spin_retains_supported_lidar(self):
  d=assess(*self.fixture(.2,.9),.8);self.assertTrue(d['icp_valid']);self.assertFalse(d['wheel_valid'])
 def test_stale_fix_cannot_adjudicate(self):
  d=assess(*self.fixture(.2,.2),2.);self.assertFalse(d['available']);self.assertFalse(d['icp_valid'])
 def test_matching_motion_is_admitted(self):
  d=assess(*self.fixture(.2,.2),.8);self.assertTrue(d['icp_valid']);self.assertTrue(d['wheel_valid'])
if __name__=='__main__':unittest.main()
