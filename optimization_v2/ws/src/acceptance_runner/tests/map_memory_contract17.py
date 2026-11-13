import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from map_memory17 import MapMemory
from dynamic_envelope17 import Envelope,to_map

class Memory(unittest.TestCase):
    def setUp(self):self.res=.1;self.origin=np.array([-5.,-5.]);self.live=np.full((100,100),-1);self.confirm=np.zeros_like(self.live,dtype=bool);self.live[70,60]=100;self.confirm[70,60]=True
    def test_fixed_world_occupancy_survives_odom_drift_without_ghosts(self):
        m=MapMemory();m.update(self.live,self.confirm,self.origin,self.res,np.eye(2),np.zeros(2),0.)
        before=set(m.cells);angle=.35;R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);tr=np.array([2.,-.5])
        for i in range(1,101):a=m.update(np.full_like(self.live,-1),np.zeros_like(self.confirm),self.origin,self.res,R,tr,float(i))
        self.assertEqual(m.cells,before);yy,xx=np.where(a==100);world=(self.origin+(np.c_[xx,yy]+.5)*self.res)@R.T+tr
        self.assertTrue(len(world)>0);self.assertLess(np.max(np.linalg.norm(world-[1.05,2.05],axis=1)),.35)
        self.assertNotEqual(a[70,60],100)
    def test_unknown_cannot_clear_but_observed_free_can(self):
        m=MapMemory();m.update(self.live,self.confirm,self.origin,self.res,np.eye(2),np.zeros(2),0.)
        a=m.update(np.full_like(self.live,-1),np.zeros_like(self.confirm),self.origin,self.res,np.eye(2),np.zeros(2),1.);self.assertTrue(m.cells)
        a=m.update(np.zeros_like(self.live),np.zeros_like(self.confirm),self.origin,self.res,np.eye(2),np.zeros(2),2.);self.assertFalse(m.cells);self.assertTrue(np.all(a==-1))
    def test_free_corners_cannot_clear_unknown_centre(self):
        m=MapMemory();m.cells={(0,0)};m.resolution=.1
        h=math.pi/4;R=np.array([[math.cos(h),-math.sin(h)],[math.sin(h),math.cos(h)]])
        tr=np.array([.05,.05])-R@np.array([.05,.05]);live=np.zeros_like(self.live);live[50,50]=-1
        m.update(live,np.zeros_like(self.confirm),self.origin,.1,R,tr,1.)
        self.assertTrue(m.cells)
    def test_unconfirmed_lethal_not_persisted(self):
        m=MapMemory();a=m.update(self.live,np.zeros_like(self.confirm),self.origin,self.res,np.eye(2),np.zeros(2),0.);self.assertFalse(m.cells)
    def test_static_object_in_drifting_odom_does_not_arm_motion(self):
        m=Envelope();world=np.array([4.,2.])
        for i in range(20):
            angle=i*.03;R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);tr=np.array([i*.05,0.]);odom=(world-tr)@R
            packet=dict(t=i*.2,frame='odom',objects=[dict(id=1,confirmed=True,observed_age_s=0.,observation_t=i*.2,position=[*odom,1.],velocity=[.5,.5],size=[.5,.5,1.8],covariance=np.eye(4).ravel().tolist())])
            m.update(to_map(packet,R,tr),i*.2)
        self.assertFalse(m.cells)
if __name__=='__main__':unittest.main()
