import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tool_sim/scripts'))
from common10 import *
from ledger_core import Ledger,latest_at
import argparse,signal
from collections import deque
import rclpy
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from mission_interfaces.srv import LedgerCommand

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=read(a.config);engine=Ledger(a.out/'ledger.sqlite',c);rclpy.init();n=rclpy.create_node('s10_work_ledger');pub=n.create_publisher(String,'/s10/ledger/state',1);contexts=deque(maxlen=100);st=dict(t=0.,wall=time.monotonic(),stop=False,last_key=None,rewound=False);log=(a.out/'events.jsonl').open('a')
 def clock(m):
  t=m.clock.sec+m.clock.nanosec*1e-9
  if t<st['t']:st['rewound']=True
  if t!=st['t']:st['wall']=time.monotonic()
  st['t']=t
 def context(m):contexts.append(json.loads(m.data))
 def observe(m):
  d=json.loads(m.data);ctx=min(contexts,key=lambda x:abs(x['t']-d['t'])) if contexts else None
  if st['rewound']:ctx=None
  result=engine.observe(d,ctx,st['t']);log.write(json.dumps(dict(t=st['t'],**result))+'\n')
 def service(req,res):
  try:
   d=json.loads(req.request_json)
   if d['op']=='reserve':r=engine.reserve(d['goal'],d['attempt'],st['t']);st['last_key']=d['goal']['idempotency_key']
   elif d['op']=='finish':r=engine.finish(d['key'],d['state'],d['reason'],d['uncertain'],st['t'])
   elif d['op']=='snapshot':r=engine.snapshot(d['key'])
   else:r=dict(ok=False,error='UNKNOWN_OPERATION')
  except Exception as ex:r=dict(ok=False,error=type(ex).__name__+':'+str(ex))
  res.response_json=json.dumps(r);return res
 def publish():
  key=engine.active or st['last_key'];r=engine.snapshot(key,geometry=False) if key else dict(ok=False)
  ctx=latest_at(contexts,st['t']);reason=engine.guard(ctx,engine.goal(key),st['t']) if key and r['ok'] else 'NO_GOAL'
  if st['rewound'] or time.monotonic()-st['wall']>c['clock_wall_s']:reason='CLOCK_WATCHDOG'
  start_reason=engine.guard(ctx,engine.goal(key),st['t'],for_start=True) if key and r['ok'] else 'NO_GOAL'
  if reason=='CLOCK_WATCHDOG':start_reason=reason
  r.update(t=st['t'],context_t=ctx['t'] if ctx else None,guard=reason,valid=reason=='OK',start_guard=start_reason,start_valid=start_reason=='OK',active=bool(engine.active));pub.publish(String(data=json.dumps(r)));dump(a.out/'live.json',r);log.flush()
 n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data);n.create_subscription(String,'/s10/context',context,30);n.create_subscription(String,'/s10/tool/feedback',observe,30);n.create_service(LedgerCommand,'/s10/ledger/command',service);n.create_timer(.05,publish)
 def stop(s,f):st['stop']=True
 signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop);(a.out/'ready').write_text('SQLite WAL FULL sole writer')
 try:
  while not st['stop']:rclpy.spin_once(n,timeout_sec=.02)
 finally:
  keys=[x[0] for x in engine.db.execute('SELECT key FROM jobs')];dump(a.out/'summary.json',dict(jobs=[engine.snapshot(k) for k in keys],rejections=engine.rejected));engine.close();log.close();n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
