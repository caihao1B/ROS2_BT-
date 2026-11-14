import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from recovery_mowing_r10 import observed_cursor

class RecoveryCursor(unittest.TestCase):
 def test_monotonic_local_progress(self):
  path=[[i*.04,0.,0.] for i in range(30)]
  self.assertEqual(observed_cursor(path,[.12,0.,0.],0),3)
  self.assertEqual(observed_cursor(path,[.04,0.,0.],3),3)
 def test_near_return_branch_cannot_skip_loop(self):
  path=[[i*.04,0.,0.] for i in range(30)]+[[.12,.001,0.]]
  self.assertEqual(observed_cursor(path,[.12,.001,0.],0),3)
 def test_distant_or_wrong_heading_does_not_advance(self):
  path=[[i*.04,0.,0.] for i in range(30)]
  self.assertEqual(observed_cursor(path,[.12,.3,0.],0),0)
  self.assertEqual(observed_cursor(path,[.12,0.,.3],0),0)
 def test_final_index_retained(self):
  self.assertEqual(observed_cursor([[0,0,0],[.04,0,0]],[.04,0,0],1),1)

if __name__=='__main__':unittest.main()
