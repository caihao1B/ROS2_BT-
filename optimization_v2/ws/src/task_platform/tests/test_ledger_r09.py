import unittest,tempfile,sys,json,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from ledger_core_r09 import Ledger
import yaml
BASE=Path('/home/ceihao/robot_project/optimization_v2')
class Accounting(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.c=yaml.safe_load((BASE/'config/tool_contract.yaml').read_text());self.c.update(vehicle='ackermann',session='synthetic-test',map_id='map',map_revision='map-r',frame_epoch='epoch',plan_revision='plan',tool_revision='tool',work_max_speed=.4,tool=dict(virtual_effective_length_m=.7,virtual_effective_width_m=1.6));self.e=Ledger(Path(self.tmp.name)/'ledger.sqlite',self.c);self.g=dict(operation=2,task_id='unit',idempotency_key='same-work',map_id='map',map_revision='map-r',frame_epoch='epoch',plan_revision='plan',tool_revision='tool',segment_id='strip',frame='map',target=[0.,0.,0.],area=[[-2.,-2.],[2.,-2.],[2.,2.],[-2.,2.]],requested_quantity=16.,timeout_sim_s=100.);self.e.reserve(self.g,'attempt',0.)
 def tearDown(self):self.e.close();self.tmp.cleanup()
 def feed(self,t,seq,segment='COVERAGE',healthy=True,x=0.):
  ctx=dict(t=t,localization=True,perception=True,target_ok=True,v=.2,w=0.,yaw=0.,tool_xy=[x,0.],segment=segment,segment_id='strip',max_speed=.4)
  m=dict(t=t,seq=seq,key='same-work',attempt='attempt',tool_revision='tool',sensor_epoch='synthetic-test',rpm_rad_s=12.,flow=1.,healthy=healthy,request_on=True)
  return self.e.observe(m,ctx,t)
 def test_turn_and_fault_never_credit(self):
  for i in range(20):self.feed(i*.05,i,segment='TURN_TOOL_OFF')
  self.assertEqual(self.e.snapshot('same-work')['confirmed_quantity'],0.)
  for i in range(20,40):self.feed(i*.05,i,healthy=False)
  self.assertEqual(self.e.snapshot('same-work')['confirmed_quantity'],0.)
 def test_stationary_overlap_does_not_double_area_and_width_is_bound(self):
  for i in range(30):self.feed(i*.05,i)
  first=self.e.snapshot('same-work')['confirmed_quantity'];self.assertGreater(first,.7)
  for i in range(30,60):self.feed(i*.05,i)
  self.assertEqual(first,self.e.snapshot('same-work')['confirmed_quantity'])
  self.assertLess(first,.7*1.6)
 def test_partial_stop_keeps_original_residual(self):
  for i in range(30):self.feed(i*.05,i,x=i*.01)
  done=self.e.finish('same-work','SUCCEEDED','segment-arrived',False,2.)
  self.assertEqual(done['state'],'PARTIAL');self.assertGreater(done['remaining_quantity'],14.)
  changed=dict(self.g,requested_quantity=1.);self.assertEqual(self.e.reserve(changed,'again',3.)['error'],'IDEMPOTENCY_CONFLICT')
if __name__=='__main__':unittest.main()

class BoundaryGeometry(unittest.TestCase):
 def test_mixed_boundary_intersection_does_not_crash_or_credit_lines(self):
  from shapely.geometry import GeometryCollection,LineString,box
  from ledger_core_r09 import add_area
  g=GeometryCollection([box(0,0,1,1),LineString([(1,1),(2,2)])]);r=add_area(box(-1,0,0,1),g,.002);self.assertAlmostEqual(r.area,2.)
 def test_only_touching_boundary_creates_no_area(self):
  from shapely.geometry import LineString,Polygon
  from ledger_core_r09 import add_area
  self.assertEqual(add_area(Polygon(),LineString([(0,0),(1,1)]),.002).area,0.)
