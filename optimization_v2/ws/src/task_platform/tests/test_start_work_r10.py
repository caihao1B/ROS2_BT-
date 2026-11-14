import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from start_work_r10 import startup_interrupt,wait_startup,working_feedback_ready

class StartWork(unittest.TestCase):
 def test_working_feedback_belongs_to_fresh_active_goal(self):
  d=dict(t=1.,active=True,key='current',working_confirmed=True)
  self.assertTrue(working_feedback_ready(d,1.01,'current'))
  for changes,now in [({'active':False},1.01),({'key':'previous'},1.01),({},1.2),({'working_confirmed':False},1.01)]:
   self.assertFalse(working_feedback_ready(dict(d,**changes),now,'current'))
 def test_all_non_execute_commands_interrupt(self):
  for action in ['CANCEL','WAIT','STOP','HUMAN','REPLAN','DEFER']:
   self.assertEqual(startup_interrupt(dict(t=10.,action=action),10.01),action)
 def test_stale_execute_is_not_admission(self):
  self.assertEqual(startup_interrupt(dict(t=9.8,action='EXECUTE'),10.),'BT_STALE')
 def test_cancel_wins_over_simultaneous_working_feedback(self):
  t=[0.]
  def tick():t[0]+=.02
  reason=wait_startup(tick,lambda:t[0],lambda:dict(t=t[0],action='CANCEL'),lambda:True)
  self.assertEqual(reason,'CANCEL');self.assertLess(t[0],.1)
 def test_mid_start_wait_returns_before_original_deadline(self):
  t=[0.]
  def tick():t[0]+=.02
  reason=wait_startup(tick,lambda:t[0],lambda:dict(t=t[0],action='WAIT' if t[0]>=.3 else 'EXECUTE'),lambda:False)
  self.assertEqual(reason,'WAIT');self.assertLess(t[0],.33)
 def test_feedback_required_and_bounded(self):
  t=[0.]
  def tick():t[0]+=.02
  with self.assertRaisesRegex(TimeoutError,'TOOL_START_DEADLINE'):
   wait_startup(tick,lambda:t[0],lambda:dict(t=t[0],action='EXECUTE'),lambda:False,seconds=.3)
  self.assertLess(t[0],.33)
 def test_healthy_feedback_admits(self):
  t=[0.]
  def tick():t[0]+=.02
  self.assertIsNone(wait_startup(tick,lambda:t[0],lambda:dict(t=t[0],action='EXECUTE'),lambda:t[0]>.2))
