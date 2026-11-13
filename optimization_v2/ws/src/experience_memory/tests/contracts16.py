from pathlib import Path
import sys,tempfile,copy,json,sqlite3,time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common16 import *
from store16 import Store
from features16 import Trigger,CausalContext

def run(out):
 cfg=config();x=dict(vector=[.5,.5,1.6,.3,0,.3,0,0,0,1,.05,.9,0,.2,0,0,0],category='person',mode='ALONG',velocity=[.3,0],age=0.,context_age=0.,sigma=.05,clearance=4.,speed=.3,accel=0.,loc_ok=True)
 checks={};evidence={}
 with tempfile.TemporaryDirectory() as tmp:
  p=Path(tmp)/'memory.sqlite';s=Store(p,cfg);s.put('train-example',x,100,'train-only unit fixture');checks['normal_hit']=s.lookup(x,101)['hit'];s.close();s=Store(p,cfg);checks['restart_preserves']=s.lookup(x,102)['hit'];checks['clock_rewind_reject']=s.lookup(x,101)['reason']=='CLOCK_REWIND'
  for name,patch,reason in [('reverse',dict(vector=x['vector'][:3]+[-.3,0]+x['vector'][5:],velocity=[-.3,0]),'OPPOSITE_DIRECTION'),('ood',dict(speed=3.),'OUT_OF_DISTRIBUTION'),('conflict',dict(clearance=.1),'CURRENT_CONFLICT'),('quality',dict(loc_ok=False),'LOCALIZATION_OR_UNCERTAINTY'),('stale',dict(age=1.),'STALE_INPUT'),('expired',{},'EXPIRED')]:
   r=s.lookup(dict(x,**patch),100+cfg['ttl_s']+1 if name=='expired' else 103);checks[name]=not r['hit'] and r['reason']==reason;evidence[name]=r
  # Reset persistent last clock to a fresh store after explicit far-future expiry test.
  s.close();p.unlink();s=Store(p,cfg);s.put('train-example',x,100,'train-only unit fixture')
  for k in ['map','model','feature']:
   old=s.version.copy();s.version[k]='different';r=s.lookup(x,104);checks[k+'_version']=not r['hit'] and r['reason']=='VERSION_MISMATCH';s.version=old
  s.cfg=dict(cfg,query_timeout_ms=0);r=s.lookup(x,105);checks['deadline_fallback']=not r['hit'] and r['reason']=='QUERY_TIMEOUT';s.cfg=cfg
  other=sqlite3.connect(p,timeout=.001);other.execute('BEGIN EXCLUSIVE');r=s.lookup(x,106);checks['locked_db_fallback']=not r['hit'] and r['reason']=='DATABASE_ERROR';other.rollback();other.close()
  s.close()
  small=dict(cfg,max_prototypes=8,max_audit_rows=16,max_store_bytes=262144);s=Store(Path(tmp)/'growth.sqlite',small);sizes=[]
  for i in range(80):s.put('row'+str(i),x,100+i,'train-pressure-test');sizes.append(s.size())
  count=s.db.execute('SELECT COUNT(*) FROM prototypes').fetchone()[0];evicted=int(s.db.execute("SELECT value FROM metadata WHERE key='evicted'").fetchone()[0]);audits=s.db.execute('SELECT COUNT(*) FROM audit').fetchone()[0]
  checks['bounded_growth']=max(sizes)<=small['max_store_bytes'] and count==8 and evicted==72 and audits==16
  s.close();s=Store(Path(tmp)/'growth.sqlite',small);checks['growth_restart']=s.db.execute('SELECT COUNT(*) FROM prototypes').fetchone()[0]==8 and s.db.execute('PRAGMA quick_check').fetchone()[0]=='ok';checks['ttl_eviction']=s.maintain(100+80+cfg['ttl_s'])==8
  evidence['growth']=dict(bytes=sizes,live_rows=count,evicted=evicted,audit_rows=audits,test_cap_bytes=small['max_store_bytes'],production_cap_bytes=cfg['max_store_bytes']);s.close()
  tr=Trigger(cfg);a=tr.event(1,0,x);b=tr.event(1,.2,x);c=tr.event(1,.4,dict(x,velocity=[-.3,0]));checks['causal_event']=a[0] and not b[0] and c[0]
 c=CausalContext();c.add_status(dict(t=1.,navigation_allowed=True));c.add_status(dict(t=1.2,navigation_allowed=False));c.add_robot(dict(t=1.01,x=1.));c.add_robot(dict(t=1.3,x=99.))
 checks['causal_context_excludes_newer_callback']=c.at(1.1)==dict(t=1.,navigation_allowed=True,robot=dict(t=1.01,x=1.)) and c.at(.5)==dict(robot={})
 result=dict(passed=all(checks.values()),checks=checks,evidence=evidence);dump(out,result)
 if not result['passed']:raise AssertionError(checks)
 return result
if __name__=='__main__':print(json.dumps(run(Path(sys.argv[1]))['checks']))
