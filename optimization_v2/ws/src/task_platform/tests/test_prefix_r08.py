import sys,unittest,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prefix_r08 import validate
class Prefix(unittest.TestCase):
 def run_case(self,g,path=None):return validate(g,[-2,-2],.05,[0,0,0],[0,0,0],[0,0,0],path or [[0,0,0],[1,0,0],[2,0,0]],[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]])
 def test_observed_prefix_only_not_whole_plan(self):
  g=np.zeros((100,100),int);g[:,75:]=-1;r=self.run_case(g);self.assertTrue(r['valid']);self.assertEqual(r['length_m'],1.)
 def test_unknown_safety_margin_blocks(self):
  g=np.zeros((100,100),int);g[45:47,51]=-1;self.assertFalse(self.run_case(g)['valid'])
 def test_inside_body_measured_obstacle_is_never_cleared(self):
  g=np.zeros((100,100),int);g[40,40]=100;self.assertFalse(self.run_case(g)['valid'])
 def test_in_place_rotation_sweep(self):
  g=np.zeros((100,100),int);g[48,40]=100;self.assertFalse(self.run_case(g,[[0,0,0],[0,1,1.5707963268]])['valid'])
if __name__=='__main__':unittest.main()
