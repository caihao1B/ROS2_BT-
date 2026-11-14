import unittest,sys,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from activity_r08 import ActivityMemory,route_conflict,apply_grid
from shapely.geometry import box
class Activity(unittest.TestCase):
 def tr(self,p):return dict(id=2,position=[*p,1.],size=[.4,.4,1.],confirmed=True,state='OBSERVED',observed_age_s=0.)
 def test_observed_history_persists_through_turn(self):
  a=ActivityMemory();a.update(0,[self.tr([3,0])],{2});g=a.update(1,[self.tr([3,2])],set());self.assertTrue(g.covers(__import__('shapely').Point(3,0)))
 def test_static_track_does_not_create_activity(self):self.assertTrue(ActivityMemory().update(0,[self.tr([3,0])],set()).is_empty)
 def test_unobserved_history_expires(self):
  a=ActivityMemory();a.update(0,[self.tr([3,0])],{2});self.assertTrue(a.update(35.1,[],set()).is_empty)
 def test_risk_never_clears_unknown(self):
  c=np.full((10,10),-1);r=apply_grid(c,[0,0],1.,box(2,2,4,4));self.assertTrue(np.all(np.isin(r,[-1,100])));self.assertEqual(r[0,0],-1)
 def test_rejoin_is_beyond_activity_not_within_it(self):
  r=route_conflict(box(1,-1,5,1),[[0,0],[12,0]],[0,0],[[-.25,-.2],[.25,.2]]);self.assertTrue(r['conflict']);self.assertEqual(r['clear_ahead_m'],7.)
 def test_clear_side_path_does_not_intersect_history(self):self.assertFalse(route_conflict(box(2,2,5,3),[[0,0],[10,0]],[0,0],[[-.25,-.2],[.25,.2]])['conflict'])

class ActiveReference(unittest.TestCase):
 def test_detour_is_measured_against_its_actual_reference(self):
  from tracking_error_r08 import errors
  e=[dict(event='ACTIVE_REFERENCE_CHANGED',t=0,epoch=0,patch=True,reference_map=[[0,1,0],[2,1,0]]),dict(event='OBSERVED_PATCH_ADMITTED',t=0,epoch=0)]
  v,ok=errors([dict(t=1,xyz=[29,15,0])],e,[[28,14],[30,14]]);self.assertTrue(ok);self.assertEqual(v,[0.])
 def test_motion_on_rejected_candidate_is_not_authorized(self):
  from tracking_error_r08 import errors
  e=[dict(event='ACTIVE_REFERENCE_CHANGED',t=0,epoch=0,patch=True,reference_map=[[0,1,0],[2,1,0]])]
  v,ok=errors([dict(t=0,xyz=[28,15,0]),dict(t=1,xyz=[29,15,0])],e,[[28,14],[30,14]]);self.assertFalse(ok)
