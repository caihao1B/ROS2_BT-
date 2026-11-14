"""Segment-aware Ackermann worker; real Nav2/ToolTask actions, feedback-only credit."""
import argparse,json,time,signal,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath,ComputePathToPose
from mission_interfaces.action import ToolTask
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped,Point32
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);out=ap.parse_args().out;cfg=json.loads((out/'mission.json').read_text());spec=json.loads((out/'spec.json').read_text());toolcfg=json.loads((out/'scene/tool.json').read_text());cluster=next(x for x in cfg['clusters'] if x['cluster']==spec['cluster']);allsegments=cluster['segments'];segments=allsegments[:spec['segment_limit']] if spec['segment_limit'] else allsegments
 rclpy.init();n=rclpy.create_node('r09_mowing_task',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)]);nav=ActionClient(n,FollowPath,'/follow_path');planner=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');tool=ActionClient(n,ToolTask,'/s10/tool_task');manager=n.create_client(Trigger,'/lifecycle_manager/is_active');pub=n.create_publisher(String,'/r07/mission',10);hintpub=n.create_publisher(PoseWithCovarianceStamped,'/s07/rtabmap/initialpose',10)
 st=dict(t=0.,stop=False,phase='STARTUP',nav_idle=True,complete=False,nav_failed=False,replans=0,deferrals=0,alternative=False,index=0,segment='TRANSIT',segment_id='',direction=1,work_complete=False,bt=None,gate=None,loc=None,tool=None);active=None;active_tool=None;tool_future=None;start=time.monotonic();hinted=False;terminal='FAULT';error=None;completed=[];work_results=[];last_progress_t=None;last_p=None
 def log(event,**d):
  with (out/'mission_events.jsonl').open('a') as f:f.write(json.dumps(dict(t=st['t'],event=event,**d))+'\n')
 n.create_subscription(Clock,'/clock',lambda m:st.update(t=m.clock.sec+m.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,key in [('/r06/directive','bt'),('/r07/gate','gate'),('/r07/localization','loc'),('/s10/action/state','tool')]:n.create_subscription(String,topic,lambda m,k=key:st.update({k:json.loads(m.data)}),20)
 def publish():
  d={k:st[k] for k in ['t','phase','nav_idle','complete','nav_failed','replans','deferrals','alternative','index','segment','segment_id','direction','work_complete']};pub.publish(String(data=json.dumps(d)));tmp=out/'live.tmp';tmp.write_text(json.dumps(dict(d,bt=st['bt'],gate=st['gate'],loc=st['loc'],tool=st['tool'])));tmp.replace(out/'live.json')
 n.create_timer(.03,publish)
 def tick(allow_stop=False):
  nonlocal hinted,last_progress_t,last_p
  rclpy.spin_once(n,timeout_sec=.01)
  if st['t']>8 and not hinted:
   h=spec['operator_hint'];m=PoseWithCovarianceStamped();m.header.frame_id='map';m.pose.pose.position.x=h[0];m.pose.pose.position.y=h[1];m.pose.pose.orientation.z=math.sin(h[2]/2);m.pose.pose.orientation.w=math.cos(h[2]/2);m.pose.covariance=[1. if i in [0,7,14] else .25 if i in [21,28,35] else 0. for i in range(36)];hintpub.publish(m);hinted=True;log('COARSE_PUBLIC_CLUSTER_HINT')
  loc=st['loc'] or {}
  if loc.get('valid'):
   if last_p is None or math.dist(last_p,loc['pose'][:2])>.1:last_p=loc['pose'][:2];last_progress_t=st['t']
  if not allow_stop:
   if st['stop']:raise InterruptedError('SESSION_STOP')
   if st['t']>spec['sim_seconds'] or time.monotonic()-start>spec['wall_budget']-30:raise TimeoutError('TASK_BUDGET')
   if active and last_progress_t is not None and st['t']-last_progress_t>45:raise TimeoutError('NO_OBSERVED_PROGRESS')
 def wait(pred,seconds=20,allow_stop=False):
  t=st['t'];wall=time.monotonic()
  while not pred():
   tick(allow_stop)
   if st['t']-t>seconds or time.monotonic()-wall>max(20,seconds*4):raise TimeoutError('WAIT_DEADLINE')
 def result(f,seconds=20):wait(f.done,seconds);return f.result()
 def pose(p):
  m=PoseStamped();m.header.frame_id='map';m.pose.position.x=float(p[0]);m.pose.position.y=float(p[1]);m.pose.orientation.z=math.sin(p[2]/2);m.pose.orientation.w=math.cos(p[2]/2);return m
 def tool_finish_result():
  nonlocal active_tool,tool_future
  if tool_future is not None:
   wait(tool_future.done,5,True);r=tool_future.result().result;d=dict(segment=st['segment_id'],state=r.state,reason=r.reason,confirmed=r.confirmed_quantity,remaining=r.remaining_quantity,uncertain=r.uncertainty_pending);work_results.append(d);log('TOOL_RESULT',**d);active_tool=None;tool_future=None
   if d['uncertain']:raise RuntimeError('TOOL_STOP_UNCONFIRMED')
   return d
 def stop_all():
  nonlocal active
  st['phase']='STOPPING';nav_cancel=active.cancel_goal_async() if active else None;tool_cancel=active_tool.cancel_goal_async() if active_tool else None
  for f in [nav_cancel,tool_cancel]:
   if f is not None:wait(f.done,5,True)
  if active:
   f=active.get_result_async();wait(f.done,5,True);active=None
  st['nav_idle']=True;tool_finish_result();wait(lambda:bool(st['gate'] and st['gate']['stopped'] and st['gate']['tool_stopped']),5,True);log('BOTH_PHYSICAL_STOPS_CONFIRMED',segment=st['segment_id'])
 def begin_tool(s):
  nonlocal active_tool,tool_future
  g=ToolTask.Goal();g.operation=ToolTask.Goal.START_WORK;g.task_id='R09_'+cluster['cluster'];g.idempotency_key=cfg['version']+':'+s['id'];g.segment_id=s['id'];g.map_id=cfg['map_id'];g.map_revision=cfg['map_revision'];g.frame_epoch=cfg['frame_epoch'];g.plan_revision=cfg['version'];g.tool_revision=toolcfg['tool_revision'];g.target=pose(s['poses'][0]);wire=np.asarray(s['target_area'],dtype=np.float32).astype(float);g.target_area.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in wire];g.requested_quantity=float(abs(np.dot(wire[:,0],np.roll(wire[:,1],-1))-np.dot(wire[:,1],np.roll(wire[:,0],-1)))/2);g.timeout_sim_s=300.
  active_tool=result(tool.send_goal_async(g),5)
  if not active_tool.accepted:active_tool=None;raise RuntimeError('TOOL_ACTION_REJECTED')
  tool_future=active_tool.get_result_async();wait(lambda:tool_future.done() or bool(st['tool'] and st['tool'].get('working_confirmed')),8)
  if tool_future.done():
   d=tool_finish_result()
   if d['state']=='SUCCEEDED':st['work_complete']=True;return
   raise RuntimeError('TOOL_ADMISSION_FAILED')
  log('TOOL_WORKING_CONFIRMED',segment=s['id'],requested_area_m2=g.requested_quantity)
 def replan_evidence(s):
  # An alternative must retain the fixed swath entry and nonholonomic geometry.
  # No silent replacement of coverage work by a free-space drive.
  st.update(phase='REPLANNING',replans=st['replans']+1);wait(planner.server_is_ready,5);g=ComputePathToPose.Goal();g.planner_id='GridBased';g.goal=pose(s['poses'][-1]);h=result(planner.send_goal_async(g),5)
  if not h.accepted:log('ACK_PLANNER_REJECTED',attempt=st['replans']);return
  r=result(h.get_result_async(),12);log('ACK_PLANNER_RESULT',attempt=st['replans'],status=r.status,candidate_poses=len(r.result.path.poses),claim='bounded current observed search only; no global impossibility claim')
  if r.status==4 and len(r.result.path.poses)>1:raise RuntimeError('ALTERNATE_ROUTE_REQUIRES_COVERAGE_RESEQUENCING_RETAIN_RESIDUAL')
 def end(s,f):st['stop']=True
 for sig in [signal.SIGINT,signal.SIGTERM]:signal.signal(sig,end)
 (out/'observer_ready').write_text('Ackermann FollowPath + actual ToolTask feedback; no actuator or truth authority')
 try:
  wait(lambda:nav.server_is_ready() and tool.server_is_ready() and bool(st['loc'] and st['loc']['valid'] and st['gate'] and st['gate']['tool_stopped']),75);wait(manager.service_is_ready,15)
  while not result(manager.call_async(Trigger.Request()),5).success:
   t=st['t'];wait(lambda:st['t']>t+.3,2)
  log('NAV_LIFECYCLE_ACTIVE_VERIFIED')
  for index,s in enumerate(segments):
   stop_all();st.update(index=index,segment=s['kind'],segment_id=s['id'],direction=s['direction'],work_complete=False,nav_failed=False,replans=0,phase='READY');done=False;attempts=0
   while not done:
    wait(lambda:bool(st['bt'] and st['bt']['action'] in ['EXECUTE','REPLAN','HUMAN','STOP']),55);action=st['bt']['action']
    if action in ['HUMAN','STOP']:raise RuntimeError('BT_'+st['bt'].get('reason_code','HOLD'))
    if action=='REPLAN':
     if st['replans']>=2:raise RuntimeError('ACK_BOUNDED_SEARCH_EXHAUSTED')
     replan_evidence(s);st['nav_failed']=True;barrier=st['t'];wait(lambda:(st['bt']or{}).get('evidence',{}).get('stamp',-1)>barrier,3);continue
    if s['kind']=='COVERAGE' and not st['work_complete']:begin_tool(s)
    points=np.asarray(s['poses']);p=np.asarray(st['loc']['pose']);i=int(np.argmin(np.linalg.norm(points[:,:2]-p[:2],axis=1)))
    if math.dist(p[:2],points[i,:2])>.4:raise RuntimeError('SEGMENT_CAPTURE_OUTSIDE_QUALIFIED_CORRIDOR')
    goal=FollowPath.Goal();goal.controller_id='RPP';goal.goal_checker_id='goal';goal.progress_checker_id='progress';goal.path.header.frame_id='map';goal.path.header.stamp=n.get_clock().now().to_msg();goal.path.poses=[pose(p)]+[pose(x) for x in points[min(i+1,len(points)-1):]];st.update(phase='NAVIGATING',nav_idle=False);active=result(nav.send_goal_async(goal),8)
    if not active.accepted:active=None;st['nav_idle']=True;raise RuntimeError('FOLLOW_PATH_REJECTED')
    log('SEGMENT_GOAL',segment=s['id'],kind=s['kind'],direction=s['direction'],original_poses=len(points),remaining_poses=len(goal.path.poses),tool_expected=s['tool_on']);f=active.get_result_async();interrupted=False;began=st['t']
    while not f.done():
     tick()
     if tool_future is not None and tool_future.done():
      d=tool_finish_result()
      if d['state']!='SUCCEEDED':raise RuntimeError('WORK_ACTION_FAILED_BEFORE_SEGMENT_ARRIVAL '+d['reason'])
      st['work_complete']=True;log('WORK_QUANTITY_CONFIRMED_BEFORE_PATH_END',segment=s['id']);wait(lambda:bool(st['gate'] and st['gate']['tool_stopped']),4)
     bt=st['bt'] or {}
     if bt.get('action') in ['CANCEL','WAIT','REPLAN','HUMAN','STOP','DEFER'] and st['t']-began>.4:log('BT_INTERRUPTS_SEGMENT',action=bt['action'],reason=bt.get('reason_code'));interrupted=True;stop_all();break
    if f.done() and f.result().status==4:
     active=None;st['nav_idle']=True;stop_all();p=st['loc']['pose'];q=points[-1];dist=math.dist(p[:2],q[:2]);angle=abs(math.atan2(math.sin(p[2]-q[2]),math.cos(p[2]-q[2])));log('SEGMENT_ARRIVAL',segment=s['id'],xy_error_m=dist,yaw_error_rad=angle)
     if dist>.10 or angle>.12:raise RuntimeError('SEGMENT_ARRIVAL_NOT_CONFIRMED')
     completed.append(s['id']);done=True
    else:
     if active:stop_all()
     st.update(nav_failed=not interrupted,phase='OBSERVATION_HOLD');barrier=st['t'];choices=['EXECUTE','REPLAN','HUMAN','STOP'] if interrupted else ['REPLAN','HUMAN','STOP'];wait(lambda:bool(st['bt'] and st['bt'].get('evidence',{}).get('stamp',-1)>barrier and st['bt']['action'] in choices),55)
    attempts+=1
    if attempts>6:raise RuntimeError('SEGMENT_RETRY_BOUND')
  st.update(complete=True,phase='DONE');log('SELECTED_SEGMENTS_TRAVERSED',selected=len(segments),original=len(allsegments),coverage_completion='read feedback ledger separately');t=st['t'];wait(lambda:st['t']>t+2,4);terminal='COMPLETED'
 except InterruptedError as ex:error=repr(ex);terminal='CANCELLED';log('TASK_STOP',error=error)
 except BaseException as ex:error=repr(ex);terminal='HUMAN_REQUIRED' if isinstance(ex,(RuntimeError,TimeoutError)) and 'STOP_UNCONFIRMED' not in str(ex) and 'CONFIRMATION_TIMEOUT' not in str(ex) else 'FAULT';log('TASK_STOP',error=error,terminal=terminal)
 finally:
  try:stop_all()
  except BaseException as ex:terminal='FAULT';error=error or repr(ex);log('STOP_UNCONFIRMED',error=repr(ex))
  ownership={topic:[i.node_name for i in n.get_publishers_info_by_topic(topic)] for topic in ['/tf','/s03/actuator/request','/s10/tool/command']};final=dict(passed=terminal=='COMPLETED' and len(completed)==len(segments),terminal=terminal,controlled_hold=terminal=='HUMAN_REQUIRED',error=error,t=st['t'],selected_segments=len(segments),original_segments=len(allsegments),completed_segments=completed,remaining_selected=[s['id'] for s in segments if s['id'] not in completed],not_selected=[s['id'] for s in allsegments[len(segments):]],work_results=work_results,replans=st['replans'],final_gate=st['gate'],ownership=ownership,wall_seconds=time.monotonic()-start,scope='Selected path segments executed; tool area and residual are separate ledger evidence, never inferred from arrival');(out/'mission_result.json').write_text(json.dumps(final,indent=2));n.destroy_node();rclpy.try_shutdown()
 return 0 if terminal in ['COMPLETED','HUMAN_REQUIRED'] else 2
if __name__=='__main__':raise SystemExit(main())
