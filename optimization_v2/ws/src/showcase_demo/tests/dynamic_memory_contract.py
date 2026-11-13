import unittest,sys
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parents[5];sys.path.insert(0,str(root/'optimization_v2/ws/src/showcase_demo/scripts'));sys.path.insert(0,str(root/'optimization_v2/ws/src/acceptance_runner/scripts'))
from dynamic_terrain_memory import DynamicTerrainMemory

class MemoryContract(unittest.TestCase):
    def test_observed_moving_history_cannot_become_permanent_terrain(self):
        m=DynamicTerrainMemory();live=np.full((30,30),-1,dtype=int);confirmed=np.zeros_like(live,dtype=bool);live[6,6]=100;live[22,22]=100;confirmed[live==100]=True
        m.update(live,confirmed,np.zeros(2),.1,np.eye(2),np.zeros(2),0)
        for t,x in [(0.,.25),(.5,.5),(1.,.75)]:
            packet=dict(t=t,frame='odom',objects=[dict(id=1,confirmed=True,observation_t=t,observed_age_s=0.,position=[x,.65,0.],velocity=[.5,0],size=[.4,.4,1.],covariance=(np.eye(4)*.0001).ravel().tolist())])
            result=m.update(live,confirmed,np.zeros(2),.1,np.eye(2),np.zeros(2),t,packet)
        self.assertGreater(m.removed_dynamic,0);self.assertEqual(result[6,6],-1);self.assertEqual(result[22,22],100);self.assertEqual(live[6,6],100)
    def test_unknown_or_unconfirmed_packet_cannot_clear_static_memory(self):
        m=DynamicTerrainMemory();live=np.full((20,20),-1);live[6,6]=100;confirmed=live==100
        first=m.update(live,confirmed,np.zeros(2),.1,np.eye(2),np.zeros(2),0)
        later=m.update(np.full_like(live,-1),np.zeros_like(confirmed),np.zeros(2),.1,np.eye(2),np.zeros(2),1,dict(t=1,frame='odom',objects=[]))
        self.assertEqual(first[6,6],100);self.assertEqual(later[6,6],100)
if __name__=='__main__':unittest.main()
