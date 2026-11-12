"""Use the real ledger engine in a temporary directory; no project ledger is modified."""
import sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'optimization_v2/ws/src/tool_sim/scripts'))
from common10 import contract
from ledger_core import Ledger
c=contract('diff');c.update(session='lesson',map_id='m',map_revision='1',frame_epoch='e',plan_revision='p')
g=dict(operation=1,task_id='demo',idempotency_key='once',frame='map',target=[0.,0.,0.],area=[],requested_quantity=2.,timeout_sim_s=20.,segment_id='s',**{k:c[k] for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']})
with tempfile.TemporaryDirectory() as d:
 ledger=Ledger(Path(d)/'work.sqlite',c);assert ledger.reserve(g,'attempt1')['confirmed_quantity']==0
 for seq in range(1,51):
  t=seq*.05;ctx=dict(t=t,v=0.,w=0.,yaw=0.,tool_xy=[0.,0.],localization=True,perception=True,target_ok=True)
  msg=dict(t=t,seq=seq,sensor_epoch='lesson',tool_revision=c['tool_revision'],key='once',attempt='attempt1',request_on=True,healthy=True,rpm_rad_s=12.,flow=1.)
  ledger.observe(msg,ctx,t)
 done=ledger.finish('once','SUCCEEDED','confirmed',False,t);assert done['state']=='SUCCEEDED'
 assert ledger.reserve(g,'attempt2')['replayed'];assert ledger.active is None
 ledger.close();ledger=Ledger(Path(d)/'work.sqlite',c);assert ledger.reserve(g,'after_restart')['replayed'];ledger.close()
 print('PASS：确认反馈入账，重复请求和重新打开数据库不重复执行。')
