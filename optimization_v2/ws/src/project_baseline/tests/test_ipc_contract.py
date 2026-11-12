import unittest
from ipc_contract import accept_reply
class ReplyTests(unittest.TestCase):
 def setUp(self):self.ctx={'mission_revision':2,'goal_id':'g2'};self.r=dict(schema='DINO-TASK-BT-RESULT/1.0',sequence=8,context=self.ctx,commands=[])
 def test_current_reply(self):self.assertTrue(accept_reply(self.r,(8,1.),self.ctx,1.2,1.1))
 def test_old_goal(self):self.assertFalse(accept_reply(self.r,(8,1.),dict(self.ctx,goal_id='g3'),1.2,1.1))
 def test_old_revision(self):self.assertFalse(accept_reply(self.r,(8,1.),dict(self.ctx,mission_revision=3),1.2,1.1))
 def test_late_calculation_even_if_just_received(self):self.assertFalse(accept_reply(self.r,(8,1.),self.ctx,1.6,1.6))
 def test_duplicate_reply_without_pending_tick(self):self.assertFalse(accept_reply(self.r,None,self.ctx,1.2,1.1))
 def test_unsupported_schema(self):self.r['schema']='OLD';self.assertFalse(accept_reply(self.r,(8,1.),self.ctx,1.2,1.1))
if __name__=='__main__':unittest.main(verbosity=2)
