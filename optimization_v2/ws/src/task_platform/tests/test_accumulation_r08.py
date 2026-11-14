import unittest,sys,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from accumulation_r08 import SurfaceWindow
class Window(unittest.TestCase):
 def test_unique_motion_and_expiry(self):
  a=SurfaceWindow(max_age=.8);p=np.array([[2.01,.01,1.01],[3.01,.01,1.01],[.01,.01,.01]])
  x,_=a.add(p,[0,0,0],[0,0,0,1],0);x,info=a.add(p-np.array([.1,0,0]),[.1,0,0],[0,0,0,1],.4)
  self.assertEqual(info['surface_cells'],2);self.assertEqual(len(x),3);self.assertAlmostEqual(x[0,0],1.91)
  x,info=a.add(np.array([[8.,0.,1.]]),[0,0,0],[0,0,0,1],1.3);self.assertEqual(info['frames'],1);self.assertEqual(len(x),1)
if __name__=='__main__':unittest.main()
