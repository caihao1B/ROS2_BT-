"""No ROS needed: VO increments must not survive loss, jumps or clock epochs."""
import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from visual_gate17 import Window
import numpy as np

class Contract(unittest.TestCase):
    def update(self,w,t,x,valid=True):
        return w.update(t,[x,0,0],[0,0,0,1],[.001]*6,valid)
    def test_constant_motion_and_variance(self):
        w=Window();self.assertIsNone(self.update(w,0,0));self.update(w,.25,.025)
        v,angular,cov=self.update(w,.5,.05)
        np.testing.assert_allclose(v,[.1,0,0]);np.testing.assert_allclose(angular,[0,0,0]);self.assertTrue((np.linalg.eigvalsh(cov)>=.0004).all())
    def test_tracking_loss_clears_baseline(self):
        w=Window();self.update(w,0,0);self.update(w,.25,.025,False)
        self.assertIsNone(self.update(w,.5,10));self.assertIsNone(self.update(w,.75,10.025))
        np.testing.assert_allclose(self.update(w,1,10.05)[0],[.1,0,0])
    def test_discontinuity_never_becomes_velocity(self):
        w=Window();self.update(w,0,0);self.update(w,.25,.025)
        self.assertIsNone(self.update(w,.5,10));self.assertFalse(w.rows)
        self.assertIsNone(self.update(w,.75,10.025))
    def test_clock_rollback_and_missing_frames(self):
        w=Window();self.update(w,5,0);self.update(w,5.25,.025)
        self.assertIsNone(self.update(w,1,.05));self.assertEqual(len(w.rows),1)
        self.assertIsNone(self.update(w,2,.15));self.assertEqual(len(w.rows),1)
    def test_invalid_quaternion_and_nonfinite(self):
        w=Window();self.update(w,0,0)
        self.assertIsNone(w.update(.5,[0,0,0],[0,0,0,0],[.001]*6,True));self.assertFalse(w.rows)
        self.assertIsNone(self.update(w,1,float('nan')));self.assertFalse(w.rows)
    def test_non_psd_uncertainty_rejected(self):
        w=Window();C=np.eye(6);C[0,1]=C[1,0]=2
        self.assertIsNone(w.update(1,[0,0,0],[0,0,0,1],C,True));self.assertFalse(w.rows)

if __name__=='__main__':unittest.main()
