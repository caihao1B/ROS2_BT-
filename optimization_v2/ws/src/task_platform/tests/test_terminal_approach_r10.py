"""A terminal tangent must preserve exact pose, gear, spacing and validation."""
import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from recovery_mowing_r10 import terminal_straight,gear_runs,validate_candidate

class TerminalApproach(unittest.TestCase):
 def test_exact_pose_and_forward_kinematics_at_wrapped_headings(self):
  for heading in [-math.pi,-2.99,0.,1.5,math.pi]:
   goal=[100.625,38.389,heading];path=terminal_straight(goal)
   self.assertEqual(path[-1],goal);self.assertEqual(len(gear_runs(path)),1);self.assertEqual(gear_runs(path)[0]['direction'],1)
   self.assertAlmostEqual(sum(math.dist(a[:2],b[:2]) for a,b in zip(path,path[1:])),1.2)
   self.assertTrue(all(math.dist(a[:2],b[:2])<=.04000001 for a,b in zip(path,path[1:])))
 def test_unsafe_tail_does_not_gain_admission(self):
  from shapely.geometry import box,mapping
  cfg=dict(allowed_prior=mapping(box(27,-1,28.6,16)),footprint_margin_m=.1,vehicle=dict(footprint=[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]],steering=dict(max_curvature=.34)))
  path=terminal_straight([0,0,0]);ok,reason=validate_candidate(path,path[0],path[-1],cfg)
  self.assertFalse(ok);self.assertIn('FOOTPRINT',reason)
 def test_bad_inputs_rejected(self):
  for goal,d in [([0,0,float('nan')],1.2),([0,0,0],1.21),([0,0,0],0)]:
   with self.assertRaises(ValueError):terminal_straight(goal,d)
if __name__=='__main__':unittest.main()
