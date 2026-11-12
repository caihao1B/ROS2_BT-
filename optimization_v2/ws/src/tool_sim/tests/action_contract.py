"""Real C++ Action server + SQLite node on domain 124; explicit synthetic sensor fixture.
This checks protocol/time/process faults, NOT Gazebo physical performance.
"""
import sys,os,tempfile,subprocess,signal,uuid
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from common10 import *
import rclpy
from rclpy.action import ActionClient
from mission_interfaces.action import ToolTask
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock

def main():
 out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True);c=contract('diff');c.update(session='component',map_id='component_map',map_revision='1',frame_epoch='epoch',plan_revision='plan');dump(out/'config.json',c);os.environ['ROS_DOMAIN_ID']='124';rclpy.init();n=rclpy.create_node('s10_protocol_fixture');pubs={k:n.create_publisher(String,'/s10/'+v,30) for k,v in [('feedback','tool/feedback'),('context','context')]};clock=n.create_publisher(Clock,'/clock',20);client=ActionClient(n,ToolTask,'/s10/tool_task');st=dict(t=0.,seq=0,command={},ledger={},mode='normal',pause=False);processes=[];handles=[];checks={};results=[];logs=[]
 def update(m,k):st[k]=json.loads(m.data)
 n.create_subscription(String,'/s10/tool/command',lambda m:update(m,'command'),30);n.create_subscription(String,'/s10/ledger/state',lambda m:update(m,'ledger'),30)
 def launch():
  for role,args in [('ledger',[str(BASE/'venvs/scene/bin/python'),str(LEDGER/'scripts/ledger_node.py'),'--out',str(out/'ledger'),'--config',str(out/'config.json')]),('server',[str(BASE/'ws/install/tool_sim/lib/tool_sim/tool_action_server'),'--ros-args','-p','config_file:='+str(out/'config.json')])]:
   f=(out/(role+str(len(processes))+'.log')).open('w');logs.append(f);p=subprocess.Popen(args,env=os.environ.copy(),stdout=f,stderr=subprocess.STDOUT);processes.append(p)
 def tick():
  if not st['pause']:st['t']+=.02
  m=Clock();m.clock.sec=int(st['t']);m.clock.nanosec=int((st['t']%1)*1e9);clock.publish(m);cmd=st['command'];st['seq']+=1;t=round(st['t'],6)-.025;on=cmd.get('mode')=='ON';fb=dict(t=t if st['mode']!='timestamp' else t-2,seq=st['seq'],sensor_epoch=c['session'],tool_revision=c['tool_revision'] if st['mode']!='version' else 'bad',boot=cmd.get('boot',''),key=cmd.get('key',''),attempt=cmd.get('attempt',''),request_on=on,healthy=True,rpm_rad_s=12. if on else 0.,flow=1. if on else 0.);ctx=dict(t=t,tool_xy=[0.,0.],yaw=0.,v=0.,w=0.,max_speed=.12,localization=True,perception=True,target_ok=True,segment='COVERAGE',segment_id='seg')
  pubs['context'].publish(String(data=json.dumps(ctx)))
  if st['mode']!='drop':pubs['feedback'].publish(String(data=json.dumps(fb)))
  end=time.monotonic()+.02
  while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.001)
 def wait(test,wall=12):
  deadline=time.monotonic()+wall
  while not test():
   if time.monotonic()>deadline:raise TimeoutError(str(st['ledger']))
   tick()
 def settle(sec=.7):end=st['t']+sec;wait(lambda:st['t']>=end)
 def goal(key):
  g=ToolTask.Goal();g.operation=1;g.task_id=key;g.idempotency_key=key;g.target.header.frame_id='map';g.target.pose.orientation.w=1.;g.requested_quantity=2.;g.timeout_sim_s=12.;g.segment_id='seg'
  for k in ['map_id','map_revision','frame_epoch','plan_revision','tool_revision']:setattr(g,k,c[k])
  return g
 def send(g):
  print('send '+g.idempotency_key,flush=True);f=client.send_goal_async(g);wait(f.done);h=f.result();assert h.accepted;return h,h.get_result_async()
 def finish(f):
  wait(f.done,18);rr=f.result();r=rr.result;d=dict(status=rr.status,state=r.state,quantity=r.confirmed_quantity,remaining=r.remaining_quantity,replayed=r.replayed,uncertain=r.uncertainty_pending,error=r.error_code);results.append(d);return d
 try:
  launch();wait(client.server_is_ready);settle();h,f=send(goal('completed'));r=finish(f);checks['real_action_success']=r['state']=='SUCCEEDED' and r['quantity']>=2
  q=r['quantity'];h,f=send(goal('completed'));r=finish(f);checks['replay_no_new_work']=r['replayed'] and r['quantity']==q
  h,f=send(goal('cancel'));wait(lambda:st['ledger'].get('confirmed_quantity',0)>.3 and st['ledger'].get('key')=='cancel');cancel=h.cancel_goal_async();r=finish(f);checks['typed_cancel']=r['state']=='CANCELLED' and r['status']==5 and 0<r['quantity']<2
  settle();processes[-2].send_signal(signal.SIGSTOP);h,pending_result=send(goal('stop_during_reserve'))
  duplicate=client.send_goal_async(goal('busy'));wait(duplicate.done);checks['mutual_exclusion']=not duplicate.result().accepted
  stop_goal=ToolTask.Goal();stop_goal.operation=3;stop_goal.timeout_sim_s=3.;stop_handle,stop_result=send(stop_goal);processes[-2].send_signal(signal.SIGCONT);r=finish(pending_result);sr=finish(stop_result);checks['stop_during_reservation']=r['state']=='CANCELLED' and sr['state']=='STOPPED' and not st['ledger'].get('active')
  settle();h,f=send(goal('crash'));wait(lambda:st['ledger'].get('key')=='crash' and st['ledger'].get('confirmed_quantity',0)>.3)
  for p in processes[-2:]:p.send_signal(signal.SIGINT)
  for p in processes[-2:]:p.wait(timeout=5)
  # Keep the ROS client alive while servers restart; DDS discovery replaces endpoints.
  settle();launch();settle(3.);wait(client.server_is_ready);print('restarted servers ready',flush=True);h,f=send(goal('completed'));r=finish(f);checks['restart_completed_replay']=r['state']=='SUCCEEDED' and r['replayed'] and r['quantity']==q
  h,f=send(goal('crash'));r=finish(f);checks['restart_unfinished_hold']=r['error']=='RECOVERY_REQUIRED' and r['uncertain'] and r['quantity']>.3
  for mode in ['version','timestamp','drop']:
   settle();h,f=send(goal(mode));wait(lambda:st['ledger'].get('key')==mode and st['ledger'].get('confirmed_quantity',0)>.3);st['mode']=mode;r=finish(f);checks[mode+'_fails_closed']=r['state']=='LOST' and r['uncertain'] and r['quantity']<2;st['mode']='normal';settle()
  h,f=send(goal('pause'));wait(lambda:st['ledger'].get('key')=='pause' and st['ledger'].get('confirmed_quantity',0)>.3);st['pause']=True;r=finish(f);checks['paused_clock_wall_stop']=r['state']=='LOST' and r['uncertain'];st['pause']=False;settle()
  h,f=send(goal('rewind'));wait(lambda:st['ledger'].get('key')=='rewind' and st['ledger'].get('confirmed_quantity',0)>.3);st['t']-=1.;r=finish(f);checks['rewind_latched_stop']=r['state']=='LOST' and r['uncertain']
 except Exception as e:checks['exception']=False;results.append(dict(error=repr(e)))
 finally:
  for p in processes:
   if p.poll() is None:p.send_signal(signal.SIGCONT);p.send_signal(signal.SIGINT)
  for p in processes:
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:p.kill();p.wait()
  for f in logs:f.close()
  dump(out/'result.json',dict(passed=all(checks.values()) and len(checks)>=12,checks=checks,results=results,fixture='synthetic tool/pose, real ROS actions + SQLite processes',sim_seconds=st['t']));n.destroy_node();rclpy.try_shutdown()
 print(json.dumps(checks));return 0 if all(checks.values()) and len(checks)>=12 else 1
if __name__=='__main__':raise SystemExit(main())
