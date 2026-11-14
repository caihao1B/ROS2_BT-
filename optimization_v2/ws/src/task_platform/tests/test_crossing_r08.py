import unittest,sys,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from crossing_r08 import crossing
class Crossings(unittest.TestCase):
 def call(self,p,v):return crossing([[0,0],[10,0]],[0,0],.4,p,v,np.eye(4)*.005,[.44,.5,1.8],[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]])
 def test_crossing_before_contact(self):self.assertTrue(self.call([1.,2.], [0,-.8])['conflict'])
 def test_separate_same_direction_follow_branch(self):self.assertFalse(self.call([3.,0.],[.15,0])['conflict'])
 def test_person_moving_away(self):self.assertFalse(self.call([1.,3.],[0,.8])['conflict'])
if __name__=='__main__':unittest.main()

class PredictionUncertainty(unittest.TestCase):
 def check(self,p,v,P=None):
  return crossing([[0,0],[10,0]],[0,0],0.,p,v,np.eye(4)*.005 if P is None else P,[.44,.5,1.8],[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]])
 def test_future_process_noise_stops_before_curved_approach(self):
  self.assertTrue(self.check([3.,2.],[-.1,-.6])['conflict'])
 def test_parallel_offset_track_is_not_assumed_to_be_a_leader(self):
  self.assertTrue(self.check([2.,1.],[.4,0.])['conflict'])
 def test_small_static_velocity_noise_does_not_invent_moving_obstacle(self):
  self.assertFalse(self.check([1.,1.],[.04,.02])['conflict'])
 def test_unconfirmed_velocity_is_not_used_as_precise_prediction(self):
  P=np.eye(4)*.005;P[2:,2:]=np.eye(2)*4.
  self.assertFalse(self.check([2.,1.],[.2,.1],P)['conflict'])

class TurningMemory(unittest.TestCase):
 def test_previous_dynamic_object_is_protected_during_slow_turn(self):
  r=crossing([[0,0],[10,0]],[0,0],.4,[2.,1.],[.05,0],np.eye(4)*.025,[.44,.5,1.8],[[-.254,-.215],[.254,-.215],[.254,.215],[-.254,.215]],recent_motion=True)
  self.assertTrue(r['conflict'])
 def test_memory_requires_observed_motion_and_expires(self):
  from crossing_r08 import MotionMemory
  m=MotionMemory();tr=dict(id=1,velocity=[.8,0],covariance=(np.eye(4)*.025).ravel().tolist(),confirmed=True,state='OBSERVED',observed_age_s=0.)
  self.assertEqual(m.update(0,[tr]),{1});tr['velocity']=[0.,0.]
  self.assertEqual(m.update(8,[tr]),{1});self.assertEqual(m.update(10.01,[tr]),set())
 def test_deleted_track_memory_is_removed(self):
  from crossing_r08 import MotionMemory
  m=MotionMemory();m.last={1:0.};self.assertEqual(m.update(1,[]),set())
