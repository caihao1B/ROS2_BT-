import sys,unittest,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from recovery_mowing_r10 import validate_candidate,gear_permitted,arm_direction

class RecoveryAdmission(unittest.TestCase):
 def setUp(self):
  self.cfg=dict(allowed_prior=dict(type='Polygon',coordinates=[[[20,10],[40,10],[40,30],[20,30],[20,10]]]),footprint_margin_m=.1,vehicle=dict(footprint=[[-.5,-.5],[1,-.5],[1,.5],[-.5,.5]],steering=dict(max_curvature=1/3)))
  self.path=[[i*.1,0,0] for i in range(11)]
 def test_straight_real_candidate(self):self.assertTrue(validate_candidate(self.path,[0,0,0],[1,0,0],self.cfg)[0])
 def test_wrong_goal(self):self.assertEqual(validate_candidate(self.path,[0,0,0],[1,0,.2],self.cfg)[1],'GOAL_MISMATCH')
 def test_no_in_place_ackermann(self):self.path.insert(1,[0,0,.1]);self.assertEqual(validate_candidate(self.path,[0,0,0],[1,0,0],self.cfg)[1],'IN_PLACE_ROTATION')
 def test_sharp_curve_rejected(self):self.path[5][2]=.1;self.assertEqual(validate_candidate(self.path,[0,0,0],[1,0,0],self.cfg)[1],'CURVATURE_EXCEEDED')
 def test_body_outside(self):
  p=[[x+11.5,y,h] for x,y,h in self.path];self.assertEqual(validate_candidate(p,p[0],p[-1],self.cfg)[1],'FOOTPRINT_OUTSIDE_PRIOR')
 def test_gear_interlock(self):
  self.assertFalse(gear_permitted(-1,1,False,True));self.assertFalse(gear_permitted(-1,1,True,False));self.assertTrue(gear_permitted(-1,1,True,True));self.assertTrue(gear_permitted(-1,-1,False,True))
 def test_reverse_turn_to_work_requires_preselection_before_tool_start(self):
  self.assertFalse(gear_permitted(-1,1,True,False))
  selected=arm_direction(-1,1,'READY',True,True)
  self.assertTrue(gear_permitted(selected,1,False,False))
 def test_no_preselection_with_motion_tool_or_recovery(self):
  for phase,stopped,tool_stopped in [('READY',False,True),('READY',True,False),('NAVIGATING',True,True)]:
   self.assertEqual(arm_direction(-1,1,phase,stopped,tool_stopped),-1)
  self.assertEqual(arm_direction(-1,0,'READY',True,True),-1)
