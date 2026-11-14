import unittest,sys,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stability_r09 import stable_corrections
class Stability(unittest.TestCase):
 def test_short_stable_window_is_not_enough_for_dino(self):self.assertFalse(stable_corrections([(i*.2,np.zeros(2),0.) for i in range(7)])[0])
 def test_long_measured_stability_admitted(self):self.assertTrue(stable_corrections([(i*.2,np.zeros(2),0.) for i in range(21)])[0])
 def test_motion_between_matching_modes_rejected(self):self.assertFalse(stable_corrections([(i*.2,np.array([.15 if i>10 else 0,0]),0.) for i in range(21)])[0])
