import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from docking_r10 import stable_dock

class Docking(unittest.TestCase):
 def test_stable_center(self):self.assertTrue(stable_dock([(i*.05,[.02,0]) for i in range(41)],[0,0])[0])
 def test_one_lucky_pose_is_not_admission(self):self.assertFalse(stable_dock([(0.,[0,0])],[0,0])[0])
 def test_approach_drift(self):self.assertFalse(stable_dock([(i*.05,[.0015*i,0]) for i in range(41)],[0,0])[0])
 def test_stable_but_offset(self):self.assertFalse(stable_dock([(i*.05,[.04,0]) for i in range(41)],[0,0])[0])
 def test_stale_gap(self):self.assertFalse(stable_dock([(i*.05,[0,0]) for i in range(21)]+[(2+i*.05,[0,0]) for i in range(21)],[0,0])[0])
 def test_last_pose_outside_margin(self):self.assertFalse(stable_dock([(i*.05,[0,0]) for i in range(40)]+[(2.,[.05,0])],[0,0])[0])
