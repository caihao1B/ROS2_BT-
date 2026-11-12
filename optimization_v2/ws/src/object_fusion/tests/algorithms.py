"""Production association/projection contracts, including ambiguous identities."""
import sys,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from tracker import Tracker
from detector import project_box,iou,unique_semantics
class Tests(unittest.TestCase):
 def obj(self,x,y=0,size=(.4,.5,1.7)):
  return dict(position=[x,y,.85],size=list(size),covariance=np.diag([.01,.01,.02]).reshape(-1).tolist(),classification='person',confidence=.8)
 def test_velocity_and_psd(self):
  tr=Tracker()
  for i in range(30):r,_=tr.update(i*.2,[self.obj(3+i*.1)])
  self.assertEqual(len(r),1);self.assertEqual(r[0]['id'],1);self.assertAlmostEqual(r[0]['velocity'][0],.5,delta=.04);self.assertGreater(np.linalg.eigvalsh(np.array(r[0]['covariance']).reshape(4,4)).min(),0)
 def test_occlusion_same_id(self):
  tr=Tracker()
  for i in range(10):tr.update(i*.2,[self.obj(3+i*.1)])
  r,_=tr.update(2.4,[]);self.assertEqual(r[0]['state'],'COASTING')
  r,_=tr.update(2.6,[self.obj(4.3)]);self.assertEqual(r[0]['id'],1)
 def test_expired_identity_is_new(self):
  tr=Tracker();tr.update(0,[self.obj(3)]);r,_=tr.update(1.3,[]);self.assertEqual(r,[]);r,_=tr.update(1.4,[self.obj(3)]);self.assertEqual(r[0]['id'],2)
 def test_ambiguous_no_forced_id(self):
  tr=Tracker();tr.update(0,[self.obj(3,-.1),self.obj(3,.1)]);r,a=tr.update(.2,[self.obj(3,0)]);self.assertEqual(a,[0]);self.assertTrue(any(t['state']=='AMBIGUOUS_COAST' for t in r));self.assertTrue(all(t['observation_t']==0 for t in r))
 def test_size_and_distance_gates(self):
  tr=Tracker();tr.update(0,[self.obj(3)]);r,_=tr.update(.2,[self.obj(3,size=(2,2,3)),self.obj(8)]);self.assertEqual(len(r),3)
 def test_rewind(self):
  tr=Tracker();tr.update(1,[])
  with self.assertRaises(ValueError):tr.update(.9,[])
 def test_projection(self):
  o=dict(minimum=[3,-.3,0],maximum=[3.5,.3,1.7]);r=project_box(o,np.eye(3),np.zeros(3),467.,320.,180.);self.assertIsNotNone(r);self.assertLess(r[0][0],320);self.assertGreater(r[0][2],320);self.assertAlmostEqual(iou(r[0],r[0]),1)
 def test_one_visual_box_one_cluster(self):
  a=self.obj(3);b=self.obj(3.2);a['semantic']=dict(detector_box=[0,0,10,10],projected_box=[0,0,10,10]);b['semantic']=dict(detector_box=[0,0,10,10],projected_box=[2,2,12,12]);r=unique_semantics([a,b]);self.assertEqual([o['classification'] for o in r],['person','unknown_obstacle'])
 def test_ambiguous_json_serializable(self):
  import json
  tr=Tracker();tr.update(0,[self.obj(3,-.1),self.obj(3,.1)]);json.dumps(tr.update(.2,[self.obj(3,0)]))
 def test_behind_camera(self):
  self.assertIsNone(project_box(dict(minimum=[-3,-.3,0],maximum=[-2,.3,1.7]),np.eye(3),np.zeros(3),467,320,180))
if __name__=='__main__':unittest.main()
