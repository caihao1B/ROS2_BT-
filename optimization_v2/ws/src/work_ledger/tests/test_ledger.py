"""Production engine regression: no simulated success constants in acceptance assertions."""
import sys,tempfile,unittest,sqlite3,json,os,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tool_sim/scripts'))
from common10 import contract
from ledger_core import Ledger,footprint,latest_at
class Accounting(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name)/'ledger.sqlite';self.cfg=contract('diff');self.cfg.update(session='test',map_id='map',map_revision='r1',frame_epoch='frame',plan_revision='plan');self.e=Ledger(self.path,self.cfg);self.goal=dict(operation=1,task_id='t',idempotency_key='k',frame='map',target=[0.,0.,0.],area=[],requested_quantity=2.,timeout_sim_s=30.,segment_id='seg',**{k:self.cfg[k] for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']});self.ctx=dict(t=0.,tool_xy=[0.,0.],yaw=0.,v=0.,w=0.,max_speed=.12,localization=True,perception=True,target_ok=True,segment='COVERAGE',segment_id='seg');self.seq=0;self.t=0.;self.e.reserve(self.goal,'a')
 def tearDown(self):self.e.close();self.tmp.cleanup()
 def sample(self,**kw):
  self.seq+=1;self.t+=.05;self.ctx['t']=self.t;m=dict(t=self.t,seq=self.seq,sensor_epoch='test',tool_revision=self.cfg['tool_revision'],key='k',attempt='a',request_on=True,healthy=True,rpm_rad_s=12.,flow=1.);m.update(kw);return self.e.observe(m,self.ctx,self.t),m
 def run_samples(self,n=30):
  for _ in range(n):self.sample()
 def quantity(self):return self.e.snapshot('k')['confirmed_quantity']
 def test_future_context_is_buffered_not_stale(self):
  h=[dict(t=1.,value='old'),dict(t=1.051,value='new')]
  self.assertEqual(latest_at(h,1.05)['value'],'old');self.assertEqual(latest_at(h,1.052)['value'],'new');self.assertIsNone(latest_at(h,.9))
 def test_accepted_is_not_completed(self):self.assertEqual(self.quantity(),0.);self.assertEqual(self.e.snapshot('k')['state'],'RUNNING')
 def test_feedback_and_duplicate(self):
  self.run_samples();q=self.quantity();_,m=self.sample();q2=self.quantity();self.assertGreater(q,0);self.assertEqual(self.e.observe(m,self.ctx,self.t)['reason'],'DUPLICATE');self.assertEqual(q2,self.quantity())
 def test_no_credit_for_invalid_guards(self):
  for k,v in [('localization',False),('perception',False),('target_ok',False),('v',1.),('w',.5),('tool_xy',[1.,0.])]:
   old=self.ctx[k];self.ctx[k]=v;self.run_samples(8);self.ctx[k]=old
  self.assertEqual(self.quantity(),0)
 def test_unhealthy_or_not_running(self):
  for kw in [dict(healthy=False),dict(rpm_rad_s=0.),dict(request_on=False)]:
   for _ in range(8):self.sample(**kw)
  self.assertEqual(self.quantity(),0)
 def test_wrong_time_version_attempt(self):
  for kw in [dict(t=-20.),dict(t=200.),dict(tool_revision='bad'),dict(sensor_epoch='bad'),dict(attempt='bad')]:
   for _ in range(8):self.sample(**kw)
  self.assertEqual(self.quantity(),0)
 def test_skew_and_nonmonotonic_sequence(self):
  self.run_samples();q=self.quantity();self.ctx['t']=-100;m=dict(t=self.t+.05,seq=100,sensor_epoch='test',tool_revision=self.cfg['tool_revision'],key='k',attempt='a',healthy=True,request_on=True,rpm_rad_s=12.,flow=1.)
  self.assertEqual(self.e.observe(m,self.ctx,self.t+.05)['reason'],'POSE_SKEW');self.assertEqual(self.quantity(),q)
 def test_success_replay_and_payload_conflict(self):
  self.run_samples(50);self.e.finish('k','SUCCEEDED','OK',False,self.t);q=self.quantity();r=self.e.reserve(self.goal,'b');self.assertTrue(r['replayed']);self.assertEqual(r['confirmed_quantity'],q);self.assertIsNone(self.e.active);g=dict(self.goal,requested_quantity=3.);self.assertEqual(self.e.reserve(g,'c')['error'],'IDEMPOTENCY_CONFLICT')
 def test_partial_retry(self):
  self.run_samples(20);q=self.quantity();self.e.finish('k','PARTIAL','JAM',False,self.t);self.assertTrue(self.e.reserve(self.goal,'b')['ok']);self.assertEqual(self.quantity(),q)
 def test_lost_blocks_blind_respray(self):
  self.run_samples();self.e.finish('k','LOST','FEEDBACK',True,self.t);self.assertEqual(self.e.reserve(self.goal,'b')['error'],'RECOVERY_REQUIRED')
 def test_restart_preserves_both(self):
  self.run_samples(50);self.e.finish('k','SUCCEEDED','OK',False,self.t);g=dict(self.goal,idempotency_key='unfinished');self.e.reserve(g,'b');self.e.close();self.e=Ledger(self.path,self.cfg);self.assertTrue(self.e.reserve(self.goal,'c')['replayed']);self.assertEqual(self.e.snapshot('unfinished')['state'],'RECOVERY_REQUIRED')
 def test_failed_transaction_atomic(self):
  self.run_samples();before=self.quantity();count=self.e.db.execute('SELECT COUNT(*) FROM events').fetchone()[0];self.e.db.execute("CREATE TRIGGER no_update BEFORE UPDATE ON jobs BEGIN SELECT RAISE(ABORT,'injected disk failure'); END")
  with self.assertRaises(sqlite3.IntegrityError):self.sample()
  self.assertEqual(self.quantity(),before);self.assertEqual(self.e.db.execute('SELECT COUNT(*) FROM events').fetchone()[0],count)
 def test_crash_wal_recovery(self):
  self.e.close();script="import sqlite3,os; c=sqlite3.connect(os.environ['S10_DB']); c.execute(\"UPDATE jobs SET quantity=0.7 WHERE key='k'\"); c.commit(); os._exit(9)";r=subprocess.run([sys.executable,'-c',script],env={**os.environ,'S10_DB':str(self.path)});self.assertEqual(r.returncode,9);self.e=Ledger(self.path,self.cfg);self.assertAlmostEqual(self.quantity(),.7);self.assertTrue(self.e.snapshot('k')['uncertainty_pending'])
 def test_binding_rejects_new_map(self):
  for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']:
   g=dict(self.goal);g[k]='changed';self.assertTrue(self.e.reserve(g,'b')['error'].startswith('VERSION_'))
 def test_micro_jitter_geometry_stays_bounded(self):
  self.e.close();self.cfg=contract('ackermann');self.cfg.update(session='test',map_id='map',map_revision='r1',frame_epoch='frame',plan_revision='plan');self.path=Path(self.tmp.name)/'jitter.sqlite';self.e=Ledger(self.path,self.cfg);self.goal.update(operation=2,area=[[-5,-2],[5,-2],[5,2],[-5,2]],requested_quantity=40.,tool_revision=self.cfg['tool_revision']);self.ctx['v']=.1;self.e.reserve(self.goal,'a')
  for i in range(300):
   self.ctx['tool_xy']=[.0002*(i%11),.00015*(i%7)];self.ctx['yaw']=.00004*(i%13);self.sample()
  blob=self.e.db.execute('SELECT credit FROM jobs').fetchone()[0];self.assertLess(len(blob),10000);self.assertGreater(self.quantity(),.3);self.assertLess(self.quantity(),.45);self.assertNotIn('credit_geojson',self.e.snapshot('k',geometry=False))
 def test_area_union_no_transit_no_gap_interpolation(self):
  self.e.close();self.cfg=contract('ackermann');self.cfg.update(session='test',map_id='map',map_revision='r1',frame_epoch='frame',plan_revision='plan');self.path=Path(self.tmp.name)/'area.sqlite';self.e=Ledger(self.path,self.cfg);self.goal.update(operation=2,area=[[-5,-2],[5,-2],[5,2],[-5,2]],requested_quantity=40.,tool_revision=self.cfg['tool_revision']);self.ctx['v']=.1;self.e.reserve(self.goal,'a');self.ctx['v']=0.;self.assertEqual(self.e.guard(self.ctx,self.goal,0.,for_start=True),'OK');self.assertEqual(self.e.guard(self.ctx,self.goal,0.),'WORK_SPEED');self.run_samples(10);self.assertEqual(self.quantity(),0.);self.ctx['v']=.1;self.run_samples();q=self.quantity();self.assertGreater(q,.3);self.run_samples();self.assertAlmostEqual(q,self.quantity());self.ctx['segment']='TURN';self.ctx['tool_xy']=[2.,0.];self.run_samples();self.assertAlmostEqual(q,self.quantity());self.ctx['segment']='COVERAGE';self.t+=1.;self.run_samples();self.assertAlmostEqual(2*q,self.quantity(),places=5);self.assertLess(self.quantity(),1.)
if __name__=='__main__':unittest.main(verbosity=2)
