import copy,json,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from groups_mowing_r10 import navigation_groups,execution_groups,passage,retry_exhausted
class Groups(unittest.TestCase):
 def setUp(self):
  self.s=json.loads(Path('/home/ceihao/robot_project/optimization_v2/three_tasks/editions/R09/mission.json').read_text())['clusters'][0]['segments']
 def test_headland_preserves_original_waypoints_and_order(self):
  g=navigation_groups(self.s[:5]);self.assertEqual([len(x['members']) for x in g],[1,3,1]);self.assertEqual(g[1]['poses'],self.s[1]['poses']+self.s[2]['poses'][1:]+self.s[3]['poses'][1:]);self.assertEqual([m['id'] for x in g for m in x['members']],[x['id'] for x in self.s[:5]])
 def test_work_regions_remain_original(self):
  g=navigation_groups(self.s);self.assertEqual([x['target_area'] for x in g if x['tool_on']],[x['target_area'] for x in self.s if x['tool_on']])
 def test_never_merge_gear_change(self):
  s=copy.deepcopy(self.s[1:4]);s[1]['direction']*=-1;self.assertEqual(len(navigation_groups(s)),3)
 def test_never_connect_a_geometric_gap(self):
  s=copy.deepcopy(self.s[1:4]);s[1]['poses'][0][0]+=.2;self.assertEqual(len(navigation_groups(s)),2)
 def test_all_original_obligations_retained(self):
  g=navigation_groups(self.s);self.assertEqual([m['id'] for x in g for m in x['members']],[x['id'] for x in self.s]);self.assertLess(len(g),len(self.s))
 def test_wrong_position_or_heading_cannot_credit_passage(self):
  self.assertTrue(passage([0,0,0],[.09,0,.11])[0]);self.assertFalse(passage([0,0,0],[.11,0,0])[0]);self.assertFalse(passage([0,0,0],[0,0,.13])[0])

 def test_primitive_mode_requires_each_original_goal(self):
  groups=execution_groups(self.s,'original_primitive_boundaries');self.assertEqual(len(groups),34)
  for g,s in zip(groups,self.s):
   self.assertEqual(g['poses'],s['poses']);self.assertEqual(g['members'],[dict(id=s['id'],endpoint=s['poses'][-1])])
 def test_unknown_execution_mode_cannot_silently_change_obligations(self):
  with self.assertRaises(ValueError):execution_groups(self.s,'skip_failed_headlands')

 def test_arrived_group_cannot_fail_retry_guard(self):self.assertFalse(retry_exhausted(7,1,True))
 def test_group_budget_preserves_original_segment_limits(self):
  self.assertFalse(retry_exhausted(18,3,False));self.assertTrue(retry_exhausted(19,3,False));self.assertTrue(retry_exhausted(7,1,False))
