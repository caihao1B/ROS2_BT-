import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from self_capture_mowing_r10 import keep_environment
from common import read,edition

class CaptureMask(unittest.TestCase):
    def setUp(self):self.cfg=read(edition('R09')/'terrain.json')
    def test_moving_body_return_masked_at_capture(self):
        own=np.array([[2.22,0.,1.]])
        warped=own+np.array([.08,0.,0.])
        self.assertFalse(keep_environment(own,self.cfg)[0])
        self.assertTrue(keep_environment(warped,self.cfg)[0])
    def test_original_low_ground_and_steps_remain(self):
        p=np.array([[2.22,0.,.04],[2.22,0.,-.1],[2.30,0.,.10],[2.22,1.50,.4],[0,0,1.4]])
        self.assertTrue(keep_environment(p,self.cfg).all())
    def test_same_r09_high_and_physical_body_contract(self):
        p=np.array([[0,0,.4],[-.6,0,1.13],[-.6,0,0.],[-.6,0,.04],[-.6,0,-.1],[0,1.42,.04],[0,1.5,.4],[0,0,1.4]])
        self.assertEqual(keep_environment(p,self.cfg).tolist(),[False,False,True,True,True,True,True,True])

if __name__=='__main__':unittest.main()
