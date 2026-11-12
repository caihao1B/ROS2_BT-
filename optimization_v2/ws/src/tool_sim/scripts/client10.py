"""Bounded diagnostic ROS Action client; targets selected from online estimates, never truth."""
from common10 import *
from ledger_core import footprint
import argparse,signal,copy
from collections import deque
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from mission_interfaces.action import ToolTask
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

def main():
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True);c=read(a.config);rclpy.init();n=rclpy.create_node('s10_scenario_client');client=ActionClient(n,ToolTask,'/s10/tool_task');segment=n.create_publisher(String,'/s10/segment',10);fault=n.create_publisher(String,'/s10/test/fault',10);jam=n.create_publisher(String,'/s10/tool/fault',10);st=dict(t=0.,stop=False,ctx=None,tool=None,ledger=None,move=False,segment='COVERAGE',last_pub=0.,history=deque(maxlen=250));events=[];goals={};lastfeedback={};f=(a.out/'events.jsonl').open('w')
 def log(d):d=dict(t=st['t'],**d);events.append(d);f.write(json.dumps(d)+'\n');f.flush()
 def update(m,k):
  st[k]=json.loads(m.data)
  if k=='ctx':st['history'].append(st[k])
 def clock(m):st['t']=m.clock.sec+m.clock.nanosec*1e-9
 n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data)
 for topic,k in [('/s10/context','ctx'),('/s10/action/state','tool'),('/s10/ledger/state','ledger')]:n.create_subscription(String,topic,lambda m,key=k:update(m,key),30)
 def tick():
  if st['stop']:raise KeyboardInterrupt
  rclpy.spin_once(n,timeout_sec=.005)
  if time.monotonic()-st['last_pub']>.035:
   st['last_pub']=time.monotonic();segment.publish(String(data=json.dumps(dict(t=st['t'],move=st['move'],segment=st['segment'],segment_id='diagnostic_strip'))))
 def wait(predicate,sim=15.,wall=35.):
  start=st['t'];deadline=time.monotonic()+wall
  while not predicate():
   if st['t']-start>sim or time.monotonic()>deadline:raise TimeoutError('client wait: '+str(st['t']))
   tick()
 def delay(dt):start=st['t'];wait(lambda:st['t']-start>=dt,dt+1,dt*4+5)
 def inject(mode,duration=1.):
  d=dict(mode=mode,until=st['t']+duration);(jam if mode=='JAM' else fault).publish(String(data=json.dumps(d)));log(dict(event='fault',**d))
 def goal(key,quantity=None,area=None):
  if key in goals:return goals[key]
  g=ToolTask.Goal();g.operation=1 if c['vehicle']=='diff' else 2;g.task_id='diagnostic_'+key;g.idempotency_key=key
  for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']:setattr(g,k,c[k])
  g.segment_id='diagnostic_strip';g.target.header.frame_id='map';center=np.median([x['tool_xy'] for x in st['history'] if st['t']-x['t']<3.],axis=0);g.target.pose.position.x=float(center[0]);g.target.pose.position.y=float(center[1]);g.target.pose.orientation.w=1.;g.requested_quantity=quantity or (2. if g.operation==1 else area.area);g.timeout_sim_s=30.
  if area is not None:g.target_area.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in list(area.exterior.coords)[:-1]]
  goals[key]=g;return g
 def feedback(m):lastfeedback.update(phase=m.feedback.phase,q=m.feedback.confirmed_quantity)
 def execute(label,g,trigger=None):
  lastfeedback.clear();log(dict(event='request',label=label,key=g.idempotency_key,operation=g.operation,target=[g.target.pose.position.x,g.target.pose.position.y],requested=g.requested_quantity));sf=client.send_goal_async(g,feedback_callback=feedback);wait(sf.done,3.,10.);h=sf.result()
  if not h.accepted:log(dict(event='result',label=label,state='REJECTED'));return None
  log(dict(event='accepted',label=label));rf=h.get_result_async();fired=False;start=st['t']
  while not rf.done():
   tick()
   if trigger and not fired and lastfeedback.get('q',0)>(.20 if c['vehicle']=='diff' else .35):
    fired=True
    if trigger=='CANCEL':h.cancel_goal_async();log(dict(event='cancel',label=label))
    elif trigger=='TURN':st['segment']='TURN';log(dict(event='segment_turn',label=label))
    else:inject(trigger,3. if trigger=='DROP_FEEDBACK' else 1.0)
   if st['t']-start>g.timeout_sim_s+6:raise TimeoutError('action deadline '+label)
  rr=rf.result();r=rr.result;d=dict(event='result',label=label,key=g.idempotency_key,action_status=rr.status,state=r.state,error=r.error_code,reason=r.reason,quantity=r.confirmed_quantity,remaining=r.remaining_quantity,replayed=r.replayed,uncertain=r.uncertainty_pending,residual=json.loads(r.residual_geojson));log(d);delay(.6);return d
 def stable_pose():
  h=[x for x in st['history'] if st['t']-x['t']<3. and x['localization']]
  if len(h)<30 or h[-1]['t']-h[0]['t']<2.:return False
  points=np.array([x['tool_xy'] for x in h]);return float(np.max(np.linalg.norm(points-np.median(points,axis=0),axis=1)))<.04
 def ensure_fault(label,g,trigger):
  for attempt in range(4):
   wait(lambda:st['ctx']['localization'] and st['ctx']['perception'] and stopped(),20,50)
   r=execute(label,g,trigger)
   if r and ((trigger=='JAM' and r['reason']=='TOOL_HEALTH') or (trigger=='CANCEL' and r['state']=='CANCELLED') or (trigger=='DROP_FEEDBACK' and r['state']=='LOST')):return r
   if not r or r['uncertain']:return r
   log(dict(event='safe_retry',label=label,attempt=attempt+1,reason=r['reason']))
  return r
 def ensure_success(label,g):
  for attempt in range(4):
   if c['vehicle']=='ackermann':st['move']=False
   wait(lambda:st['ctx'] and st['ctx']['localization'] and st['ctx']['perception'] and 0<=st['t']-st['ctx']['t']<.15 and stopped(),20,50)
   if c['vehicle']=='ackermann':
    st['move']=True
   r=execute(label,g)
   if c['vehicle']=='ackermann':st['move']=False
   if r and r['state']=='SUCCEEDED':return r
   if not r or r['uncertain']:return r
   log(dict(event='safe_retry',label=label,attempt=attempt+1,reason=r['reason']))
  return r
 def stopped():return st['tool'] and st['tool'].get('stopped_confirmed') and not st['tool'].get('active')
 def stop(s,f):st['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);(a.out/'ready').write_text('typed action client')
 result=dict(completed=False)
 try:
  wait(lambda:client.server_is_ready() and st['ctx'] and st['ctx']['localization'] and st['ctx']['perception'] and stopped() and stable_pose(),70,160);delay(.2)
  if c['vehicle']=='diff':
   ensure_success('success',goal('success'));execute('duplicate',goal('success'))
   bad=copy.deepcopy(goal('success'));bad.requested_quantity+=1;execute('conflict',bad)
   bad=goal('not_docked');bad.target.pose.position.x+=2;execute('not_docked',bad)
   for mode in ['LOCALIZATION','SPEED','TARGET']:
    g=goal('invalid_'+mode);inject(mode,6.);delay(.1);execute('invalid_'+mode,g);inject('CLEAR',0.);delay(1.)
   ensure_fault('partial',goal('partial'),'JAM');delay(1.);wait(stopped);ensure_success('retry_partial',goal('partial'))
   ensure_fault('cancel',goal('cancel'),'CANCEL');ensure_fault('lost',goal('lost'),'DROP_FEEDBACK');delay(2.);wait(stopped);execute('lost_retry_hold',goal('lost'))
  else:
   st['move']=True
   x,y=st['ctx']['tool_xy'];yaw=st['ctx']['yaw'];area=footprint(x+.48*math.cos(yaw),y+.48*math.sin(yaw),yaw,1.35,.60)
   g=goal('strip',area=area);execute('mow_cancel',g,'CANCEL');st['move']=False;delay(1.);wait(stopped)
   st['move']=True;ensure_success('mow_retry',g);st['move']=False;delay(1.);execute('mow_duplicate',g)
   st['segment']='TRANSIT';st['move']=True;wait(lambda:st['ctx']['v']>.025,5,15);execute('transit_no_work',goal('transit',area=area));st['segment']='TURN';execute('turn_no_work',goal('turn',area=area));st['move']=False
  g=ToolTask.Goal();g.operation=3;g.timeout_sim_s=3.;execute('stop_work',g);result['completed']=True
 except KeyboardInterrupt:result['error']='INTERRUPTED'
 except Exception as e:result['error']=repr(e)
 finally:
  st['move']=False;segment.publish(String(data=json.dumps(dict(t=st['t'],move=False,segment='TRANSIT',segment_id='diagnostic_strip'))));result.update(t=st['t'],events=events,vehicle=c['vehicle']);dump(a.out/'result.json',result);f.close()
 # Keep alive until owned runtime stops: no essential process disappears mid-session.
 try:
  while not st['stop']:tick()
 except KeyboardInterrupt:pass
 # Reuse the already-discovered Action client: a fresh CLI client can take seconds to discover.
 shutdown=dict(state='UNCONFIRMED',uncertainty_pending=True)
 try:
  g=ToolTask.Goal();g.operation=3;g.timeout_sim_s=3.;sf=client.send_goal_async(g);deadline=time.monotonic()+2.8
  while not sf.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
  if sf.done() and sf.result().accepted:
   rf=sf.result().get_result_async()
   while not rf.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
   if rf.done():shutdown=dict(state=rf.result().result.state,uncertainty_pending=rf.result().result.uncertainty_pending)
 except Exception as exc:shutdown['error']=repr(exc)
 dump(a.out/'shutdown.json',shutdown);n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
