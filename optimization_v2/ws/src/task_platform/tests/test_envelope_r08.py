import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from path_envelope_r08 import envelope_free
class ObservedEnvelope(unittest.TestCase):
 fp=[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]]
 def check(self,c,p,pose=(0,0,0)):return envelope_free(c,p,pose,self.fp)
 def test_current_occupied_footprint_unknown_only(self):self.assertTrue(self.check([-1,-1],[[0,0],[.2,0]]))
 def test_unknown_future_never_admitted(self):self.assertFalse(self.check([0,-1],[[0,0],[.4,0]]))
 def test_measured_lethal_inside_current_never_cleared(self):self.assertFalse(self.check([100],[[0,0]]))
 def test_clearance_margin_is_not_occupied(self):self.assertFalse(self.check([-1],[[.3,0]]))
 def test_known_continuation(self):self.assertTrue(self.check([0,80],[[0,0],[1,0]]))
 def test_pose_rotation(self):
  import math
  self.assertFalse(self.check([-1],[[3,4.4]],(3,4,math.pi/2)))
  self.assertTrue(self.check([-1],[[3,4.2]],(3,4,math.pi/2)))
if __name__=='__main__':unittest.main()
