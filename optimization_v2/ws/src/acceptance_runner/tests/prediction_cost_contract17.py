import unittest,sys,numpy as np,copy
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from prediction_cost17 import apply

class Contract(unittest.TestCase):
    def setUp(self):
        self.x=np.arange(11)-5.;self.grid=np.zeros((11,11),dtype=np.int16)
        self.packet=dict(frame='odom',status='OK',t=10.,forecasts=[dict(t=10.,dt=.4,size=[.5,.5,1.5],modes=[dict(probability=1.,mean=[[0.,0.]],covariance=[[[.01,0],[0,.01]]])])])
    def test_preserves_obstacles_unknown_and_only_adds_soft_cost(self):
        self.grid[5,5]=100;self.grid[4,5]=-1
        self.assertGreater(apply(self.grid,self.x,self.x,self.packet,10.1),0)
        self.assertEqual(self.grid[5,5],100);self.assertEqual(self.grid[4,5],-1)
        self.assertTrue(np.all(self.grid[self.grid!=100]<=90));self.assertEqual(self.grid[0,0],0)
    def test_stale_future_and_wrong_frame_no_cost(self):
        for now in [9.9,11.]:self.assertEqual(apply(self.grid,self.x,self.x,self.packet,now),0)
        self.packet['frame']='map';self.assertEqual(apply(self.grid,self.x,self.x,self.packet,10.1),0)
        self.assertFalse(self.grid.any())
    def test_invalid_covariance_does_not_create_a_forecast(self):
        self.packet['forecasts'][0]['modes'][0]['covariance']=[[[.01,2],[2,.01]]]
        self.assertEqual(apply(self.grid,self.x,self.x,self.packet,10.1),0)
    def test_overlapping_mode_cost_is_bounded(self):
        mode=self.packet['forecasts'][0]['modes'][0];mode['probability']=.5
        self.packet['forecasts'][0]['modes'].append(copy.deepcopy(mode))
        self.assertGreater(apply(self.grid,self.x,self.x,self.packet,10.1),0);self.assertEqual(self.grid.max(),90)
if __name__=='__main__':unittest.main()
