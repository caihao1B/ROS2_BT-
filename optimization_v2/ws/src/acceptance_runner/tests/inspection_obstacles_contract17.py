import unittest,sys,math
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from inspection_obstacles17 import conflicts

def pose(x,y,yaw=0):return S(pose=S(pose=S(position=S(x=x,y=y,z=0),orientation=S(x=0,y=0,z=math.sin(yaw/2),w=math.cos(yaw/2)))))
class Contract(unittest.TestCase):
    def setUp(self):
        self.item=dict(id=1,position=[4,1,0],velocity=[.4,0],confirmed=True,observed_age_s=0)
        self.s=dict(t=10,pose=pose(20,30),odom=pose(0,0),inspection_objects=dict(frame='odom',t=9.9,objects=[self.item]));self.path=[[20,30,0],[28,30,0]]
    def test_frame_transform_and_reference_distance(self):
        d=conflicts(self.s,self.path);self.assertEqual(len(d),1);self.assertAlmostEqual(d[0]['distance_to_reference_m'],1)
    def test_rotated_origin_is_not_confused_with_map(self):
        self.s['pose']=pose(20,30,math.pi/2);self.path=[[20,30,0],[20,38,0]]
        self.assertAlmostEqual(conflicts(self.s,self.path)[0]['distance_to_reference_m'],1)
    def test_old_future_wrong_frame_unconfirmed_static(self):
        for field,value in [('t',8),('t',11),('frame','map')]:
            old=self.s['inspection_objects'][field];self.s['inspection_objects'][field]=value
            self.assertFalse(conflicts(self.s,self.path));self.s['inspection_objects'][field]=old
        self.item['confirmed']=False;self.assertFalse(conflicts(self.s,self.path));self.item['confirmed']=True
        self.item['velocity']=[.01,0];self.assertFalse(conflicts(self.s,self.path))
    def test_far_from_segment_no_trigger(self):
        self.item['position']=[4,3,0];self.assertFalse(conflicts(self.s,self.path))
if __name__=='__main__':unittest.main()
