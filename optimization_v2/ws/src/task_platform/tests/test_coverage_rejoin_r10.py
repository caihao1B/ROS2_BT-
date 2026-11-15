import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from coverage_rejoin_r10 import rejoin_candidates,forward_window

class CoverageRejoin(unittest.TestCase):
 def setUp(self):self.points=[[i*.1,0.,0.] for i in range(231)]
 def test_forward_candidates_keep_original_pose_and_bounded_advance(self):
  before=[p[:] for p in self.points];r=rejoin_candidates(self.points,[.35,.02,0.])
  self.assertEqual(len(r),6);self.assertTrue(all(0<q['advance_m']<=12.15 for q in r));self.assertTrue(all(q['index']>q['original_cursor'] for q in r));self.assertEqual(self.points,before)
  self.assertTrue(all(q['pose']==self.points[q['index']] for q in r))
 def test_short_tail_cannot_invent_work_or_extend_original_strip(self):
  self.assertEqual(rejoin_candidates(self.points,[22.,0.,0.]),[])
  self.assertEqual(forward_window(self.points,225)[-1],self.points[-1])
 def test_resume_window_is_local_prefix_not_whole_obligation(self):
  p=forward_window(self.points,30);self.assertEqual(p[0],self.points[30]);self.assertLessEqual(math.dist(p[0][:2],p[-1][:2]),1.600001)
 def test_original_endpoint_length_bound_remains_twelve_metres(self):
  from recovery_mowing_r10 import validate_candidate
  cfg=dict(allowed_prior=dict(type='Polygon',coordinates=[[[0,0],[100,0],[100,100],[0,100],[0,0]]]),footprint_margin_m=.1,vehicle=dict(footprint=[[-.5,-.5],[1,-.5],[1,.5],[-.5,.5]],steering=dict(max_curvature=1/3)))
  points=self.points[:151]
  self.assertEqual(validate_candidate(points,points[0],points[-1],cfg)[1],'RECOVERY_TOO_LONG')
  self.assertTrue(validate_candidate(points,points[0],points[-1],cfg,25.)[0])
