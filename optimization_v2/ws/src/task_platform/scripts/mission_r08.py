"""Independent inspection action client. Completion means measured route arrival."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath,ComputePathToPose
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args();out=a.out;cfg=json.loads((out/'mission.json').read_text());spec=json.loads((out/'spec.json').read_text());full=np.asarray(cfg['route_map']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(full[:,:2],axis=0),axis=1))];limit=spec['distance_limit_m'];count=len(full) if limit<=0 else max(2,int(np.searchsorted(arc,limit,side='right')));route=full[:count];selected=float(arc[count-1]);rclpy.init();n=rclpy.create_node('r08_inspection_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);nav=ActionClient(n,FollowPath,'/follow_path');planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');manager=n.create_client(Trigger,'/lifecycle_manager/is_active');pub=n.create_publisher(String,'/r07/mission',10);hp=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10);st=dict(t=0.,stop=False,phase='STARTUP',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=False,index=0,bt=None,gate=None,loc=None);startwall=time.monotonic();active=None;hinted=False;error=None;manual_reason=None;events=[];trace=(out/'inspection_trace.jsonl').open('w')
 def log(event,**d):
  row=dict(t=st['t'],event=event,**d);events.append(row)
  with (out/'mission_events.jsonl').open('a') as f:f.write(json.dumps(row)+'\n')
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,key in [('/r06/directive','bt'),('/r07/gate','gate'),('/r07/localization','loc')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
 def publish():
  loc=st['loc'] or {}
  if loc.get('valid') and 'pose' in loc:
   lo=max(0,st['index']-2);hi=min(len(route),st['index']+40);i=lo+int(np.argmin(np.linalg.norm(route[lo:hi,:2]-np.array(loc['pose'][:2]),axis=1)));st['index']=max(st['index'],i);trace.write(json.dumps(dict(t=st['t'],pose=loc['pose'],route_index=i,estimated_arc_m=float(arc[i])))+'\n');trace.flush()
  d={k:st[k] for k in ['t','phase','nav_idle','complete','nav_failed','replans','deferrals','alternative','index']};d.update(selected_length_m=selected,original_length_m=float(arc[-1]));pub.publish(String(data=json.dumps(d)));tmp=out/'live.tmp';tmp.write_text(json.dumps(dict(d,bt=st['bt'],gate=st['gate'],loc=st['loc'])));tmp.replace(out/'live.json')
 n.create_timer(.03,publish)
 def tick(allow_stop=False):
  nonlocal hinted
  rclpy.spin_once(n,timeout_sec=.01)
  if st['t']>8 and not hinted:
   h=spec['operator_hint'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=h[0];m.pose.pose.position.y=h[1];m.pose.pose.orientation.z=math.sin(h[2]/2);m.pose.pose.orientation.w=math.cos(h[2]/2);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hp.publish(m);hinted=True;log('COARSE_PUBLIC_ROUTE_HINT')
  if not allow_stop and (st['stop'] or st['t']>spec['sim_seconds'] or time.monotonic()-startwall>spec['wall_budget']-30):raise TimeoutError('TASK_BOUND_OR_STOP')
 def wait(pred,seconds=20,allow_stop=False):
  begin=st['t'];wall=time.monotonic()
  while not pred():
   tick(allow_stop)
   if st['t']-begin>seconds or time.monotonic()-wall>max(20,seconds*4):raise TimeoutError('WAIT_DEADLINE')
 def result(f,seconds=20):wait(f.done,seconds);return f.result()
 def cancel():
  nonlocal active
  if active:
   f=active.cancel_goal_async();wait(f.done,8,True);log('FOLLOW_PATH_CANCEL_ACK',accepted=bool(f.result() and f.result().goals_canceling));f=active.get_result_async();wait(f.done,8,True);active=None
  st.update(nav_idle=True,phase='STOPPING');wait(lambda:bool(st['gate'] and st['gate']['stopped']),5,True)
 def pose(x,y,h):
  p=PoseStamped();p.header.frame_id='map';p.pose.position.x=float(x);p.pose.position.y=float(y);p.pose.orientation.z=math.sin(h/2);p.pose.orientation.w=math.cos(h/2);return p
 def path_goal():
  p=st['loc']['pose'];i=st['index']
  if math.dist(p[:2],route[i,:2])>.6:raise RuntimeError('OUTSIDE_QUALIFIED_ROUTE_CORRIDOR')
  g=FollowPath.Goal();g.controller_id='RPP';g.goal_checker_id='goal';g.progress_checker_id='progress';g.path.header.frame_id='map';g.path.poses=[pose(*p)]+[pose(*q) for q in route[min(i+1,len(route)-1):]];return g
 def replan_query():
  nonlocal manual_reason
  wait(planner.server_is_ready,5);g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=pose(*route[-1]);h=result(planner.send_goal_async(g),5)
  if not h.accepted:log('PLANNER_QUERY_REJECTED');manual_reason='PLANNER_QUERY_REJECTED';return
  r=result(h.get_result_async(),15);log('PLANNER_QUERY_RESULT',status=r.status,poses=len(r.result.path.poses),candidate_admission='NOT_ADMITTED_WITHOUT_CURRENT_OBSERVED_DETOUR_VALIDATION');manual_reason='LIVE_DETOUR_INTEGRATION_PENDING' if r.status==4 else 'BOUNDED_PLANNER_FAILED_NO_GLOBAL_INFEASIBILITY_PROOF'
 def stop(s,f):st['stop']=True
 for s in [signal.SIGINT,signal.SIGTERM]:signal.signal(s,stop)
 (out/'observer_ready').write_text('real FollowPath action; no tool or actuator publisher')
 try:
  wait(lambda:nav.server_is_ready() and bool(st['loc'] and st['loc']['valid']) and bool(st['gate'] and st['gate']['tool_stopped']),75);wait(manager.service_is_ready,15)
  begin=st['t']
  while not result(manager.call_async(Trigger.Request()),5).success:
   if st['t']-begin>35:raise TimeoutError('NAV_STARTUP')
   t=st['t'];wait(lambda:st['t']>t+.3,2)
  log('NAV_LIFECYCLE_ACTIVE_VERIFIED')
  while not st['complete']:
   st['phase']='READY';wait(lambda:bool(st['bt'] and st['bt']['action'] in ['EXECUTE','FOLLOW','REPLAN','HUMAN','STOP']),60)
   if st['bt']['action']=='REPLAN':replan_query();raise RuntimeError(manual_reason)
   if st['bt']['action'] in ['HUMAN','STOP']:raise RuntimeError('BT_'+st['bt'].get('reason_code','HOLD'))
   goal=path_goal();st.update(phase='NAVIGATING',nav_idle=False);active=result(nav.send_goal_async(goal),10)
   if not active.accepted:active=None;st.update(nav_idle=True);raise RuntimeError('FOLLOW_PATH_REJECTED')
   log('FOLLOW_PATH_GOAL',start_index=st['index'],poses=len(goal.path.poses),selected_length_m=selected);future=active.get_result_async();interrupted=False;started=st['t']
   while not future.done():
    tick();bt=st['bt'] or {}
    if bt.get('action') in ['CANCEL','WAIT','REPLAN','HUMAN','STOP','DEFER'] and st['t']-started>.5:
     log('BT_INTERRUPTS_PATH',action=bt['action'],reason=bt.get('reason_code'));interrupted=True;cancel();break
   if future.done() and future.result().status==4:
    active=None;st.update(nav_idle=True,phase='STOPPING');wait(lambda:bool(st['gate'] and st['gate']['stopped']),5);distance=math.dist(st['loc']['pose'][:2],route[-1,:2]);st['complete']=distance<.2 and st['index']>=len(route)-5;log('FOLLOW_PATH_SUCCESS',endpoint_error_m=distance)
    if not st['complete']:raise RuntimeError('ARRIVAL_NOT_MEASURED')
   else:
    if active:cancel()
    st.update(phase='OBSERVATION_HOLD' if interrupted else 'NAV_FAILED',nav_failed=not interrupted)
    failure_stamp=st['t'];allowed_next=['EXECUTE','FOLLOW','REPLAN','HUMAN','STOP'] if interrupted else ['REPLAN','HUMAN','STOP']
    log('AWAIT_FRESH_BT_AFTER_PATH_RESULT',interrupted=interrupted,minimum_context_stamp=failure_stamp,allowed_next=allowed_next)
    wait(lambda:bool(st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>failure_stamp and st['bt']['action'] in allowed_next),60)
    if st['bt']['action']=='REPLAN':replan_query();raise RuntimeError(manual_reason)
    if st['bt']['action'] in ['HUMAN','STOP']:raise RuntimeError('BT_'+st['bt'].get('reason_code','HOLD'))
    st['nav_failed']=False;log('RESUME_REMAINING_ROUTE',index=st['index'])
  st['phase']='DONE';end=st['t'];wait(lambda:st['t']>end+2,4)
 except BaseException as ex:error=repr(ex);log('TASK_STOP',error=error,manual_reason=manual_reason)
 finally:
  st['phase']='STOPPING'
  try:cancel()
  except BaseException as ex:error=error or repr(ex);log('STOP_UNCONFIRMED',error=repr(ex))
  ownership={topic:[i.node_name for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/s03/actuator/request','/s10/tool/command']};final=dict(passed=error is None and st['complete'],error=error,manual_reason=manual_reason,t=st['t'],selected_length_m=selected,original_length_m=float(arc[-1]),full_route_selected=count==len(full),remaining_original_m=max(0,float(arc[-1])-float(arc[st['index']])),route_index=st['index'],final_gate=st['gate'],ownership=ownership,wall_seconds=time.monotonic()-startwall,scope='Inspection navigation; no spray/mow credit');(out/'mission_result.json').write_text(json.dumps(final,indent=2));trace.close();n.destroy_node();rclpy.try_shutdown()
 return 0 if final['passed'] else 2
if __name__=='__main__':raise SystemExit(main())
