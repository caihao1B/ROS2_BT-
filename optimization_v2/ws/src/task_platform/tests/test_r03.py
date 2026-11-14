import sys,unittest,copy,tempfile,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common import *
from verify_r03 import validate
from reeds_r03 import query,sample
class Routes(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.m=read(edition('R03')/'mission.json')
 def test_01_valid(self):self.assertTrue(validate(self.m)['passed'])
 def test_02_missing_cluster(self):
  m=copy.deepcopy(self.m);m['task_plans']['mowing'].pop();self.assertFalse(validate(m)['checks']['mowing_plan_count'])
 def test_03_disconnected_path(self):
  m=copy.deepcopy(self.m);m['task_plans']['mowing'][0]['segments'][1]['poses'][0][0]+=.5;self.assertFalse(validate(m)['plans']['B1']['checks']['connected'])
 def test_04_wrong_gear(self):
  m=copy.deepcopy(self.m);s=next(s for s in m['task_plans']['mowing'][0]['segments'] if s.get('direction')==-1);s['direction']=1;self.assertFalse(validate(m)['plans']['B1']['checks']['gear'])
 def test_05_cross_boundary(self):
  m=copy.deepcopy(self.m);m['task_plans']['spray'][0]['segments'][0]['poses'][0][0]=-100;self.assertFalse(validate(m)['plans']['A1']['checks']['footprint'])
 def test_06_native_reverse_endpoint(self):
  with tempfile.TemporaryDirectory() as d:
   for goal in [[-4.,0,0],[0,1.4,math.pi]]:
    q=query([0,0,0],[goal],3.1,Path(d))[0];chunks=sample([0,0,0],q['primitives']);end=chunks[-1]['poses'][-1];self.assertLess(math.dist(end[:2],goal[:2]),1e-6);self.assertLess(abs(math.atan2(math.sin(end[2]-goal[2]),math.cos(end[2]-goal[2]))),1e-6);self.assertTrue(any(c['direction']==-1 for c in chunks))
 def test_07_budget_and_hash(self):
  m=copy.deepcopy(self.m);m['task_plans']['inspection'][0]['estimated_seconds']=1801;self.assertFalse(validate(m)['plans']['road']['checks']['budget']);self.assertFalse(validate(m)['checks']['manifest_version'])
if __name__=='__main__':unittest.main()
