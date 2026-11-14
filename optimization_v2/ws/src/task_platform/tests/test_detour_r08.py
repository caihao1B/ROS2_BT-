import sys,unittest,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from detour_r08 import plan
class ObservedDetour(unittest.TestCase):
 fp=[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]]
 def run_plan(self,g):return plan(g,[0.,0.],.1,[1.,3.,0.],[(60,[7.,3.])],self.fp,max_wall_s=2.)
 def test_obstacle_detour_has_clearance(self):
  g=np.zeros((60,90),dtype=int);g[25:35,35:45]=100;r=self.run_plan(g);self.assertTrue(r['valid']);self.assertGreater(max(abs(p[1]-3) for p in r['poses']),.9);self.assertGreater(r['length_m'],6.)
 def test_unknown_wall_does_not_become_free(self):
  g=np.zeros((60,90),dtype=int);g[:,35:45]=-1;r=self.run_plan(g);self.assertFalse(r['valid']);self.assertFalse(r.get('global_no_path_proof',False))
 def test_narrow_gap_rejects_body_not_only_centerline(self):
  g=np.zeros((60,90),dtype=int);g[:,35:45]=100;g[28:32,35:45]=0;self.assertFalse(self.run_plan(g)['valid'])
 def test_unknown_start_is_not_force_cleared(self):
  g=np.zeros((60,90),dtype=int);g[29:31,9:11]=-1;self.assertEqual(self.run_plan(g)['reason'],'START_FOOTPRINT_NOT_OBSERVED')
 def test_bounded_search_does_not_claim_global_infeasible(self):
  g=np.zeros((60,90),dtype=int);r=plan(g,[0,0],.1,[1,3],[(0,[7,3])],self.fp,max_expansions=1);self.assertEqual(r['reason'],'BOUNDED_LOCAL_SEARCH_EXHAUSTED')
if __name__=='__main__':unittest.main()
