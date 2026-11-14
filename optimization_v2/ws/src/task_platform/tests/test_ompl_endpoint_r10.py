import sys,unittest,tempfile,math,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ompl_endpoint_r10 import launch,sample,APP
from recovery_mowing_r10 import validate_candidate,yaw_error

class ContinuousEndpoint(unittest.TestCase):
 def solve(self,start,goal):
  with tempfile.TemporaryDirectory() as tmp:
   child,response=launch(start,goal,3.0983388881649963,Path(tmp));stdout,stderr=child.communicate(timeout=3);self.assertEqual(child.returncode,0,stderr.decode());return sample(start,response)
 def test_near_goal_with_lateral_and_heading_error(self):
  a=[82.63716224,35.10208627,.30417459];b=[82.65590848,35.04331222,.26179939];points,detail=self.solve(a,b);cfg=json.loads((APP/'editions/R10/mowing/mission.json').read_text());self.assertTrue(validate_candidate(points,a,b,cfg)[0]);self.assertLess(math.dist(points[-1][:2],b[:2]),1e-7);self.assertLess(yaw_error(points[-1][2],b[2]),1e-7);self.assertGreater(len(set(detail['gear_sequence'])),1)
 def test_same_position_is_not_an_in_place_yaw_jump(self):
  a=[0.,0.,0.];b=[0.,0.,.09];points,detail=self.solve(a,b);self.assertGreater(detail['length_m'],.2)
  for x,y in zip(points,points[1:]):
   ds=math.dist(x[:2],y[:2]);self.assertLessEqual(2*math.sin(yaw_error(x[2],y[2])/2)/ds,1/3.0983388881649963+1e-8)
 def test_angle_wrap_reaches_exact_orientation(self):
  a=[0.,0.,3.13];b=[.02,-.06,-3.12];points,_=self.solve(a,b);self.assertLess(math.dist(points[-1][:2],b[:2]),1e-7);self.assertLess(yaw_error(points[-1][2],b[2]),1e-7)
