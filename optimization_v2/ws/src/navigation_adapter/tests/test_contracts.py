"""Negative data-fusion contracts; no simulation pass inferred from unit tests."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prior_cost13 import fuse,in_map,np
from analysis13 import tracking
from calibrate13 import fit
class Contracts(unittest.TestCase):
 def test_unknown_not_blindly_free(self):
  live=np.array([[-1,-1,100,0,-1]]);prior=np.array([[-1,0,0,100,0]]);memo=np.array([[-1,-1,-1,-1,100]])
  self.assertEqual(fuse(live,prior,memo).tolist(),[[-1,0,100,100,100]])
 def test_independent_tracking(self):
  e,h=tracking(np.array([[.5,.1],[1.5,-.2]]),np.array([0.,.1]),[[0,0,0],[2,0,0]])
  self.assertTrue(np.allclose(e,[.1,.2]));self.assertTrue(np.allclose(h,[0,.1]))
 def test_map_reprojection_preserves_unknown_and_hazard(self):
  grid=np.full((6,6),-1);grid[1,2]=100;grid[4,4]=0
  rot=np.array([[0.,-1.],[1.,0.]])
  out,origin=in_map(grid,[0,0],1.,rot,np.array([10.,20.]),[7.,23.])
  # Occupied source center (2.5,1.5) -> (8.5,22.5).
  i,j=np.floor((np.array([8.5,22.5])-origin)).astype(int)
  self.assertEqual(out[j,i],100)
  self.assertTrue(np.any(out==-1));self.assertLessEqual(np.count_nonzero(out==0),1)
  self.assertTrue(np.all(origin==[4.,20.]))
 def test_heldout_registration_refuses_outlier(self):
  def point(x,dy=0):return dict(map_xy=[x,0.],site_xy=[x+.4,-.1+dy],map_scatter_p95_m=.02)
  train=[point(x) for x in [0,3,6]];held=[point(1.5),point(4.5)]
  self.assertTrue(fit(train,held)['passed'])
  held[0]=point(1.5,1.);self.assertFalse(fit(train,held)['passed'])
if __name__=='__main__':unittest.main()
