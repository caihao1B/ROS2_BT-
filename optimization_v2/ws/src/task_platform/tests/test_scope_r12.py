import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from stage_r12 import declared_scope_matches

class Scope(unittest.TestCase):
 def test_startup_fault_is_not_mislabeled_as_nominal(self):
  receipt={'case':'mowing_startup_perception_loss','task':'mowing'};spec={'task':'mowing','cluster':'B1','case':'startup_perception_loss','segment_limit':5,'sim_seconds':420}
  self.assertTrue(declared_scope_matches(receipt,spec,'mowing_startup_perception_loss'))
  self.assertFalse(declared_scope_matches(receipt,dict(spec,case='nominal'),'mowing_startup_perception_loss'))
 def test_actual_runtime_distance_key(self):
  self.assertTrue(declared_scope_matches({'case':'inspection_leader','task':'inspection'},{'task':'inspection','case':'leader','distance_limit_m':20.,'sim_seconds':150},'inspection_leader'))
 def test_subset_cannot_replace_full_connector(self):
  self.assertFalse(declared_scope_matches({'case':'inspection','task':'inspection'},{'task':'inspection','case':'nominal','distance_limit_m':20.,'sim_seconds':480},'inspection'))
 def test_different_task_or_time_budget_rejected(self):
  receipt={'case':'inspection_blocked','task':'inspection'};spec={'task':'inspection','case':'blocked','distance_limit_m':20.,'sim_seconds':120}
  self.assertTrue(declared_scope_matches(receipt,spec,'inspection_blocked'))
  self.assertFalse(declared_scope_matches(receipt,dict(spec,sim_seconds=500),'inspection_blocked'))
  self.assertFalse(declared_scope_matches(receipt,dict(spec,task='mowing'),'inspection_blocked'))
 def test_partial_mowing_cannot_replace_full_cluster(self):
  receipt={'case':'mowing_full_B1','task':'mowing'};spec={'task':'mowing','cluster':'B1','case':'nominal','segment_limit':5,'sim_seconds':1800}
  self.assertFalse(declared_scope_matches(receipt,spec,'mowing_full_B1'))
  self.assertTrue(declared_scope_matches(receipt,dict(spec,segment_limit=0),'mowing_full_B1'))
 def test_offset_pilot_cannot_replace_full_cluster(self):
  receipt={'case':'mowing_full_B1','task':'mowing'};spec={'task':'mowing','cluster':'B1','case':'nominal','segment_limit':0,'segment_start':12,'sim_seconds':1800}
  self.assertFalse(declared_scope_matches(receipt,spec,'mowing_full_B1'))

 def test_wrong_cluster_and_diagnostic_profile_do_not_enter_release(self):
  receipt={'case':'spray_full_A1','task':'spray'}
  spec={'task':'spray','cluster':'A1','case':'nominal','point_limit':0,'sim_seconds':1800}
  self.assertTrue(declared_scope_matches(receipt,spec,'spray_full_A1'))
  for change in ({'cluster':'A2'},{'diagnostic_only':True},{'candidate_profile':'fixture'}):
   self.assertFalse(declared_scope_matches(receipt,dict(spec,**change),'spray_full_A1'))

if __name__=='__main__':unittest.main()
