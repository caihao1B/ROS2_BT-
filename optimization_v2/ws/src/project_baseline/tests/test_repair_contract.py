import copy,unittest
from repair_contract import request,unavailable,validate_reply
class RepairTests(unittest.TestCase):
 def setUp(self):
  self.s=dict(mission_id='m',plan_revision='p',map_revision='map',geometry_sha256='g',remaining_segment_ids=['c1','c2'],coverage_ledger_revision='l',blocked_region_revision='b')
  self.r=request('D1',self.s,'r1',10.);self.result=unavailable(self.r)
 def test_missing_ledger_binding_rejected(self):
  self.s['coverage_ledger_revision']=None
  with self.assertRaisesRegex(ValueError,'MISSING'):request('D2',self.s,'r',10.)
 def test_duplicate_obligation_rejected(self):
  self.s['remaining_segment_ids']=['c1','c1']
  with self.assertRaisesRegex(ValueError,'SEGMENT_IDS'):request('D1',self.s,'r',10.)
 def test_delayed_reply_rejected(self):
  with self.assertRaisesRegex(ValueError,'DEADLINE'):validate_reply(self.r,self.result,11.,self.s)
 def test_revised_remaining_set_rejected(self):
  self.s=copy.deepcopy(self.s);self.s['remaining_segment_ids'].pop()
  with self.assertRaisesRegex(ValueError,'STALE'):validate_reply(self.r,self.result,5.,self.s)
 def test_unknown_schema_rejected(self):
  self.result['schema']='OLD'
  with self.assertRaisesRegex(ValueError,'SCHEMA'):validate_reply(self.r,self.result,5.,self.s)
 def test_not_implemented_cannot_execute(self):
  self.assertIsNone(self.result['proposal'])
  with self.assertRaisesRegex(ValueError,'NOT_EXECUTABLE'):validate_reply(self.r,self.result,5.,self.s)
 def test_ok_does_not_bypass_future_commit_validator(self):
  self.result['status']='OK'
  with self.assertRaisesRegex(ValueError,'COMMIT_NOT_IMPLEMENTED'):validate_reply(self.r,self.result,5.,self.s)
if __name__=='__main__':unittest.main(verbosity=2)
