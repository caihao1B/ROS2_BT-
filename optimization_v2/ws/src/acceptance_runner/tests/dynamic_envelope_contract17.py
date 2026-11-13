"""Motion evidence, expiry, unknown retention and bounded memory contracts."""
import sys,unittest,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from dynamic_envelope17 import Envelope

def packet(t,x,velocity=.5):
    return dict(t=t,frame='map',objects=[dict(id=1,confirmed=True,observed_age_s=0.,observation_t=t,position=[x,0,1],velocity=[velocity,0],size=[.4,.3,1.8],covariance=np.diag([.0036,.0036,.1,.1]).ravel().tolist())])

class Contracts(unittest.TestCase):
    def test_static_jitter_does_not_leave_dynamic_trail(self):
        e=Envelope()
        for i in range(20):e.update(packet(i*.2,.01*(-1)**i,0),i*.2)
        self.assertFalse(e.cells)
    def test_observed_movement_is_retained_and_expires(self):
        e=Envelope()
        for i in range(8):e.update(packet(i*.2,i*.1),i*.2)
        self.assertTrue(e.cells);e.update(None,10);self.assertTrue(e.cells)
        e.update(None,48);self.assertFalse(e.cells)
    def test_rewind_clears_old_epoch(self):
        e=Envelope()
        for i in range(8):e.update(packet(i*.2,i*.1),i*.2)
        e.update(None,0);self.assertFalse(e.cells)
    def test_never_clears_obstacles_or_unknown_to_free(self):
        e=Envelope()
        for i in range(8):e.update(packet(i*.2,i*.1),i*.2)
        c=np.full((30,30),-1);c[0,0]=100;old=c.copy();e.apply(c,np.linspace(-3,3,30),np.linspace(-3,3,30))
        self.assertTrue(np.any(c==100));self.assertEqual(c[0,0],100);self.assertTrue(np.all((old!=-1)|(c==-1)|(c==100)))
    def test_stale_and_wrong_frame_rejected(self):
        e=Envelope()
        for i in range(8):
            p=packet(i*.2,i*.1);e.update(p,i*.2+2)
        self.assertFalse(e.cells)
        p=packet(4,2);p['frame']='truth';e.update(p,4);self.assertFalse(e.cells)
if __name__=='__main__':unittest.main()
