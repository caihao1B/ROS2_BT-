import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from progress_spray_r10 import NearGoalProgress
POLICY=dict(near_radius_m=.25,minimum_improvement_m=.005,stall_s=8.,minimum_translation_command_s=2.,maximum_sample_gap_s=.15)
class NearGoal(unittest.TestCase):
 def test_near_circling_requests_bounded_replan(self):
  p=NearGoalProgress(POLICY);events=[p.update(i*.05,[.1,0],[0,0],True,.01) for i in range(180)];self.assertTrue(any(events))
 def test_continuous_approach_is_not_a_stall(self):
  p=NearGoalProgress(POLICY);self.assertFalse(any(p.update(i*.05,[.24-i*.001,0],[0,0],True,.02) for i in range(180)))
 def test_initial_in_place_alignment_is_not_a_stall(self):
  p=NearGoalProgress(POLICY);self.assertFalse(any(p.update(i*.05,[.1,0],[0,0],True,0.) for i in range(250)))
 def test_unqualified_or_stale_poses_cannot_trigger(self):
  for valid,dt in [(False,.05),(True,.2)]:
   p=NearGoalProgress(POLICY);self.assertFalse(any(p.update(i*dt,[.1,0],[0,0],valid,.02) for i in range(250)))
 def test_far_obstacle_wait_is_not_endpoint_recovery(self):
  p=NearGoalProgress(POLICY);self.assertFalse(any(p.update(i*.05,[1.,0],[0,0],True,.02) for i in range(250)))
