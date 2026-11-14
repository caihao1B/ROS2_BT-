"""Observed stability must not admit a slow initial drift or angular ambiguity."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from startup_r08 import stable_corrections
class StartupStability(unittest.TestCase):
 def history(self,xy=None,yaw=None):
  return [(i*.2,np.array([20.,28.])+np.array((xy or [[.001*(-1)**j,0] for j in range(7)])[i]),(yaw or [-.463]*7)[i]) for i in range(7)]
 def test_stable_observed_window(self):self.assertTrue(stable_corrections(self.history())[0])
 def test_slow_translation_drift_rejected(self):self.assertFalse(stable_corrections(self.history([[i*.025,0] for i in range(7)]))[0])
 def test_angular_drift_rejected(self):self.assertFalse(stable_corrections(self.history(yaw=[i*.003 for i in range(7)]))[0])
 def test_short_window_rejected(self):self.assertFalse(stable_corrections(self.history()[:4])[0])
 def test_wrapped_heading_not_false_jump(self):self.assertTrue(stable_corrections(self.history(yaw=[np.pi-.001 if i%2 else -np.pi+.001 for i in range(7)]))[0])
 def test_nonfinite_rejected(self):self.assertFalse(stable_corrections(self.history([[float('nan'),0]]*7))[0])
if __name__=='__main__':unittest.main()
