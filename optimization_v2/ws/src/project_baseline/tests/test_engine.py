"""Drive the real compiled BT.CPP runtime, including asynchronous status and halt traces."""
import copy,json,pathlib,subprocess,time,unittest
ROOT=pathlib.Path(__file__).resolve().parents[1]
WS=ROOT.parents[1]
class Engine:
    def __init__(self):
        self.p=subprocess.Popen([str(WS/'install/project_baseline/lib/project_baseline/baseline_bt'),str(ROOT/'fixtures/tree.xml')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1);self.seq=0
        self.b=dict(wall=1.,sim=1.,phase='TRACKING',segment='c1',goal_id='goal1',role='FOLLOW_SEGMENT',fault=False,hold=False,finished=False,wait_pending=False,rejoin=False,may_execute=True,ready=True,stop_confirmed=False,yielding=False,transfer=False,repair='',reason='CLEAR')
    def tick(self,**changes):
        self.seq+=1;self.b['wall']+=.1;self.b['sim']+=.1;self.b.update(changes)
        self.p.stdin.write(json.dumps(dict(schema='DINO-TASK-BT-TICK/1.0',sequence=self.seq,context={'revision':1},snapshot=self.b))+'\n');self.p.stdin.flush()
        line=self.p.stdout.readline()
        if not line:raise RuntimeError(self.p.stderr.read())
        return json.loads(line)
    def close(self):
        self.p.stdin.close();self.p.wait(timeout=3);self.p.stdout.close();self.p.stderr.close()
class Tests(unittest.TestCase):
    def setUp(self):self.e=Engine()
    def tearDown(self):self.e.close()
    def verbs(self,r):return [c['verb'] for c in r['commands']]
    def test_real_nodes_tick_running(self):
        r=self.e.tick();self.assertEqual(r['status'],'RUNNING');self.assertEqual(r['mode'],'NORMAL_COVERAGE');self.assertIn('ALLOW',self.verbs(r));self.assertTrue(any(x.get('node')=='ExecuteSegment' and x.get('to')=='RUNNING' for x in r['transitions']))
    def test_yield_preserves_same_executor_goal(self):
        self.e.tick();r=self.e.tick(yielding=True);self.assertEqual(r['mode'],'YIELD');self.assertNotIn('PAUSE',self.verbs(r));self.assertFalse(any(x.get('event')=='HALT' for x in r['transitions']))
    def test_transfer_semantics(self):
        r=self.e.tick(transfer=True);self.assertEqual(r['mode'],'TRANSFER');self.assertIn('ALLOW',self.verbs(r))
    def test_fault_preempts_and_halts_running_action(self):
        self.e.tick();r=self.e.tick(fault=True,reason='LOCALIZATION_LOST');self.assertEqual(r['mode'],'FAILURE_PAUSE');self.assertIn('PAUSE',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r));self.assertTrue(any(x.get('event')=='HALT' and x['kind']=='ExecuteSegment' for x in r['transitions']))
    def test_wait_does_not_resume_before_cancellation_confirmed(self):
        self.e.tick();r=self.e.tick(wait_pending=True,ready=False,may_execute=False);self.assertIn('PAUSE',self.verbs(r))
        r=self.e.tick(ready=True);self.assertNotIn('RESUME',self.verbs(r));self.assertNotIn('WAIT_FINISHED',self.verbs(r))
        r=self.e.tick(phase='PAUSED',goal_id='',stop_confirmed=True);self.assertIn('WAIT_FINISHED',self.verbs(r))
    def test_wait_timeout_latches_no_unsafe_fallback(self):
        self.e.tick(wait_pending=True,ready=False,may_execute=False,phase='PAUSED',goal_id='',stop_confirmed=True)
        r=self.e.tick(sim=47.);self.assertIn('LATCH_FAILURE',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
    def test_rejoin_runs_until_follow_segment_accepted(self):
        r=self.e.tick(rejoin=True,phase='PAUSED',stop_confirmed=True,goal_id='');self.assertIn('RESUME',self.verbs(r));self.assertNotIn('REJOIN_FINISHED',self.verbs(r))
        r=self.e.tick(phase='ENTERING',role='PLAN_ENTRY',stop_confirmed=False);self.assertIn('ALLOW',self.verbs(r));self.assertNotIn('REJOIN_FINISHED',self.verbs(r))
        r=self.e.tick(phase='TRACKING',role='FOLLOW_SEGMENT');self.assertIn('REJOIN_FINISHED',self.verbs(r))
    def test_preemption_during_cancel_no_allow(self):
        self.e.tick();self.e.tick(wait_pending=True,ready=False,phase='PAUSING')
        r=self.e.tick(fault=True,reason='NEW_FAULT',phase='PAUSING');self.assertNotIn('ALLOW',self.verbs(r));self.assertNotIn('RESUME',self.verbs(r))
    def test_hold_after_restart_needs_explicit_release(self):
        for _ in range(3):
            r=self.e.tick(hold=True,phase='PAUSED',stop_confirmed=True);self.assertNotIn('RESUME',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
        r=self.e.tick(hold=False,rejoin=True);self.assertIn('RESUME',self.verbs(r))
    def test_confirmed_hold_can_last_beyond_cancel_timeout(self):
        self.e.tick(hold=True,phase='PAUSED',stop_confirmed=True)
        r=self.e.tick(wall=90.);self.assertNotIn('LATCH_FAILURE',self.verbs(r));self.assertEqual(r['status'],'RUNNING')
    def test_requested_retry_is_deferred_to_bt_stop_barrier(self):
        self.e.tick(fault=True,phase='FAILED',stop_confirmed=True)
        r=self.e.tick(fault=False,rejoin=True,retry_pending=True);self.assertIn('RESUME',self.verbs(r))
    def test_repair_interfaces_do_not_fake_success(self):
        for repair in ['D1','D2']:
            r=self.e.tick(repair=repair,may_execute=False,phase='PAUSED',stop_confirmed=True)
            self.assertIn('REPAIR_UNAVAILABLE',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
    def test_task_success_is_not_coverage_success(self):
        r=self.e.tick(finished=True,phase='COMPLETED',stop_confirmed=True,may_execute=False);self.assertEqual(r['status'],'SUCCESS');self.assertEqual(r['reason'],'COVERAGE_VERDICT_REMAINS_INDEPENDENT')
    def test_repair_requires_confirmed_stop_then_runs_async(self):
        self.e.tick();r=self.e.tick(repair='D1',repair_available=True,repair_state='IDLE',may_execute=False)
        self.assertIn('PAUSE',self.verbs(r));self.assertNotIn('START_REPAIR',self.verbs(r))
        r=self.e.tick(phase='PAUSED',stop_confirmed=True,goal_id='');self.assertIn('START_REPAIR',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
        r=self.e.tick(repair_state='RUNNING');self.assertEqual(r['status'],'RUNNING');self.assertNotIn('START_REPAIR',self.verbs(r))
        r=self.e.tick(repair_state='OK');self.assertIn('REPAIR_FINISHED',self.verbs(r))
    def test_repair_fault_halts_worker_without_dispatch(self):
        self.e.tick(repair='D2',repair_available=True,repair_state='RUNNING',phase='PAUSED',stop_confirmed=True)
        r=self.e.tick(fault=True);self.assertIn('CANCEL_REPAIR',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
    def test_repair_failed_cannot_report_task_success(self):
        r=self.e.tick(repair='D1',repair_available=True,repair_state='FAILED',phase='PAUSED',stop_confirmed=True)
        self.assertIn('REPAIR_FAILED',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
    def test_segment_timeout_halts(self):
        self.e.tick();r=self.e.tick(wall=243.);self.assertIn('LATCH_FAILURE',self.verbs(r));self.assertIn('PAUSE',self.verbs(r))
    def test_long_u_budget_does_not_disable_preemption(self):
        self.e.tick();r=self.e.tick(wall=150.);self.assertIn('ALLOW',self.verbs(r));self.assertNotIn('LATCH_FAILURE',self.verbs(r))
        r=self.e.tick(fault=True,reason='PERCEPTION_STALE');self.assertIn('PAUSE',self.verbs(r));self.assertNotIn('ALLOW',self.verbs(r))
    def test_invalid_schema_process_rejects(self):
        self.e.p.stdin.write('{"schema":"OLD"}\n');self.e.p.stdin.flush();self.assertEqual(self.e.p.wait(timeout=3),2)
    def test_old_tick_sequence_rejected(self):
        self.e.tick();self.e.p.stdin.write(json.dumps(dict(schema='DINO-TASK-BT-TICK/1.0',sequence=1,context={},snapshot=self.e.b))+'\n');self.e.p.stdin.flush();self.assertEqual(self.e.p.wait(timeout=3),2)
if __name__=='__main__':unittest.main(verbosity=2)
