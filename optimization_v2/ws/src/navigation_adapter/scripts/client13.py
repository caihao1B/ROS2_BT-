"""Real Nav2 Action client; estimated pose only; feedback-bound progress, no work credit."""
from common13 import *
import argparse,signal,copy
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from nav2_msgs.action import FollowPath,NavigateToPose
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry,OccupancyGrid,Path as ROSPath
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from PIL import Image as PILImage

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--vehicle',required=True);ap.add_argument('--controller',required=True);ap.add_argument('--case',required=True);ap.add_argument('--seconds',type=float,required=True);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True);m=plan();rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);n=rclpy.create_node('s13_action_client');follow=ActionClient(n,FollowPath,'/follow_path');navigate=ActionClient(n,NavigateToPose,'/navigate_to_pose');lease=n.create_publisher(String,'/s13/lease',1);pathpub=n.create_publisher(ROSPath,'/s13/path',2);st=dict(t=0.,stop=False,global_=None,health=None,perception=None,tool=None,pose=None,odom=None,gate=None,encoders=None,image=None,cloud=None);active=False;operation='FOLLOW';action_id='';curvature=0.;last_pub=-1e9;events=[];current_handle=None;current_result=None;completed=False;f=(a.out/'events.jsonl').open('w');routes=[];progress={};map_binding=sha(BASE/'maps/frozen/manifest.json');inspection_reference=None;inspection_distances=None;seen_checkpoints=set();binding=sha(BASE/'plans/validated/manifest.json')
 def log(event,**kw):
  d=dict(t=st['t'],event=event,**kw);events.append(d);f.write(json.dumps(d,separators=(',',':'))+'\n');f.flush()
 def update(m,k):st[k]=json.loads(m.data)
 n.create_subscription(Clock,'/clock',lambda x:st.update(t=x.clock.sec+x.clock.nanosec*1e-9),qos_profile_sensor_data)
 for topic,k in [('/s08/status','global_'),('/s05/safety_limit','health'),('/s09/safety','perception'),('/s10/action/state','tool'),('/s13/gate','gate'),('/s13/ownership_audit','audit'),('/s03/encoders','encoders')]:n.create_subscription(String,topic,lambda x,key=k:update(x,key),30)
 for typ,topic,k,q in [(PoseWithCovarianceStamped,'/s08/pose','pose',20),(Odometry,'/s05/odometry','odom',30),(Image,'/s04/perception/left','image',qos_profile_sensor_data),(PointCloud2,'/s04/perception/points','cloud',qos_profile_sensor_data)]:n.create_subscription(typ,topic,lambda x,key=k:st.update({key:x}),q)
 for topic,key in [('/local_costmap/costmap','local_map'),('/global_costmap/costmap','global_map')]:n.create_subscription(OccupancyGrid,topic,lambda msg,k=key:st.update({k:msg}),2)
 def stop(s,fr):st['stop']=True
 signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop);(a.out/'ready').write_text('real NavigateToPose/FollowPath clients; no world/truth subscription')
 def tick():
  nonlocal last_pub
  if st['stop']:raise KeyboardInterrupt
  if st['t']>=a.seconds-6:raise TimeoutError('CASE_SIM_DEADLINE')
  rclpy.spin_once(n,timeout_sec=.005)
  if time.monotonic()-last_pub>.035:
   last_pub=time.monotonic();lease.publish(String(data=json.dumps(dict(t=st['t'],active=active,operation=operation,curvature=curvature,action_id=action_id,plan_revision=m['plan_revision']))))
 def wait(pred,sim=20,wall=90):
  t=st['t'];deadline=time.monotonic()+wall
  while not pred():
   if st['t']-t>sim or time.monotonic()>deadline:raise TimeoutError(f'WAIT_EXPIRED {st["t"]:.3f}, gate={st["gate"]}, global={st["global_"]}')
   tick()
 def delay(s):t=st['t'];wait(lambda:st['t']-t>=s,s+2,s*6+10)
 def stopped():return st['odom'] is not None and abs(st['odom'].twist.twist.linear.x)<.01 and abs(st['odom'].twist.twist.angular.z)<.02
 def frame_ok():
  # Paths are fixed in map, not in a one-time odom transform. S08 owns live TF.
  return st['pose'] is not None and st['pose'].header.frame_id=='map'
 def owners_ok():
  return all([x.node_name for x in n.get_publishers_info_by_topic(topic)]==[owner] for topic,owner in [('/s13/nav_cmd','controller_server'),('/s03/actuator/request','s03_drive_bridge'),('/s03/request','s03_test_driver'),('/s13/lease','s13_action_client')])
 def admitted():
  g=st['global_'] or {};p=st['perception'] or {};h=st['health'] or {};tool=st['tool'] or {};return g.get('navigation_allowed') and not g.get('plan_revalidation_required') and p.get('geometry_fresh') and p.get('boundary_valid') and p.get('max_speed',0)>0 and h.get('max_speed',0)>0 and tool.get('stopped_confirmed') and not tool.get('active') and st['pose'] and st['odom'] and owners_ok() and frame_ok()
 def make_path(poses):
  p=ROSPath();p.header.frame_id='map'  # zero stamp requests latest TF for static geometry
  for x,y,h in poses:
   q=PoseStamped();q.header=p.header;q.pose.position.x=float(x);q.pose.position.y=float(y);q.pose.orientation.z=math.sin(h/2);q.pose.orientation.w=math.cos(h/2);p.poses.append(q)
  return p
 def xy():p=st['pose'].pose.pose;return np.array([p.position.x,p.position.y])
 def local_xy():p=st['odom'].pose.pose;return np.array([p.position.x,p.position.y])
 def costmaps_ready():
  for key,frame,point in [('local_map','odom',local_xy()),('global_map','map',xy())]:
   grid=st.get(key)
   if grid is None or grid.header.frame_id!=frame or not -.05<=st['t']-stamp(grid)<.6:return False
   i=int(math.floor((point[0]-grid.info.origin.position.x)/grid.info.resolution));j=int(math.floor((point[1]-grid.info.origin.position.y)/grid.info.resolution))
   if not (0<=i<grid.info.width and 0<=j<grid.info.height) or not 0<=grid.data[j*grid.info.width+i]<99:return False
  return True
 def checkpoint(label):
  wait(lambda:st['image'] is not None and st['cloud'] is not None and all(-.05<=st['t']-stamp(st[k])<.3 for k in ['image','cloud']),1.,5.)
  im=st['image'];cloud=st['cloud'];record=dict(label=label,pose_map=xy().tolist(),pose_odom=local_xy().tolist(),camera_age_s=st['t']-stamp(im) if im else None,lidar_age_s=st['t']-stamp(cloud) if cloud else None,lidar_points=int(cloud.width*cloud.height) if cloud else 0,observations_current=bool(im and cloud and -.05<=st['t']-stamp(im)<.3 and -.05<=st['t']-stamp(cloud)<.3))
  if im:
   arr=np.frombuffer(bytes(im.data),dtype=np.uint8).reshape(im.height,im.step)[:,:im.width*3].reshape(im.height,im.width,3)
   if im.encoding=='bgr8':arr=arr[:,:,::-1]
   p=a.out/(label+'.png');PILImage.fromarray(arr).save(p);record.update(image_file=p.name,image_sha256=sha(p))
  log('checkpoint',**record)
 def align(k):
  nonlocal active,operation,curvature,action_id
  if a.vehicle!='ackermann':return
  wait(stopped,3,12);active=True;operation='ALIGN';curvature=k;action_id='steering_alignment';start=st['t'];start_xy=local_xy().copy()
  target=[math.atan(k/(1-.35*k)),math.atan(k/(1+.35*k))]
  log('align_request',curvature=k,target=target)
  wait(lambda: st['encoders'] is not None and st['encoders']['armed'] and max(abs(x-y) for x,y in zip(st['encoders']['q'][2:],target))<.025,3,15)
  log('align_complete',elapsed_s=st['t']-start,displacement_m=float(np.linalg.norm(local_xy()-start_xy)),measured=st['encoders']['q'][2:]);active=False;operation='FOLLOW';delay(.3)
 def audit_ready():
  a=st.get('audit') or {};return a.get('ready',False) and 0<=st['t']-a.get('t',-100)<.8
 def await_nav_active():
  from nav_lifecycle13 import bringup_after_restart
  bringup_after_restart(n,tick,lambda:st['t'],audit_ready,log)
 def revalidate():
  if not frame_ok() or sha(BASE/'maps/frozen/manifest.json')!=map_binding:raise ValueError('MAP_EPOCH_CHANGED_REPLAN_REQUIRED')
  if sha(BASE/'plans/validated/manifest.json')!=binding:raise ValueError('PLAN_CHANGED')
  wait(lambda:admitted() and costmaps_ready(),35,120);wait(stopped,3,15)
 def follow_resuming(label,poses,nav=False,cancel_distance=None,preempt_distance=None):
  remainder=poses;outcome=None;did_preempt=False;terminal_retries=0
  for attempt in range(12):
   revalidate();outcome=execute(label if not attempt else label+'_resume_'+str(attempt),remainder,nav=nav,cancel_distance=cancel_distance,preempt_distance=preempt_distance if not did_preempt else None);did_preempt=did_preempt or outcome.get('preempted',False)
   if outcome['status']==4:return outcome
   if cancel_distance is not None and outcome.get('stop_reason')=='REQUESTED_CANCEL':return outcome
   if outcome.get('stop_reason')=='PATH_DEPARTURE':raise ValueError('PATH_DEPARTURE_REPLAN_REQUIRED')
   terminal_retry=outcome.get('error_code')==105 and stopped() and terminal_retries<1 and math.dist(xy(),remainder[-1][:2])<.08 and len(outcome['progress']['remaining_poses'])<=3
   if terminal_retry:
    terminal_retries+=1;log('terminal_pose_reacquisition',id=label,position_error_m=math.dist(xy(),remainder[-1][:2]),reason='STOPPED_NEAR_GOAL_PROGRESS_FAILURE',remaining=outcome['progress']['remaining_poses'])
   planning_wait=nav and outcome.get('error_code') in [202,205,206,208]
   if outcome['status']!=5 and outcome.get('error_code') not in [102,104,107] and not planning_wait and not terminal_retry:raise ValueError('NONRECOVERABLE_ACTION '+str(outcome))
   if planning_wait:
    previous_stamp=stamp(st['global_map']);log('planning_wait_for_observation',id=label,error_code=outcome['error_code'],previous_map_stamp=previous_stamp)
    wait(lambda:stamp(st['global_map'])>previous_stamp+.1 and costmaps_ready(),10,35)
   remainder=outcome['progress']['remaining_poses']
   if len(remainder)<2:raise ValueError('RESIDUAL_PATH_EMPTY_WITHOUT_SUCCESS')
   dump(a.out/'resume_checkpoint.json',dict(plan_revision=m['plan_revision'],remaining_poses=remainder,previous=outcome['progress'],no_work_credit=True));log('resume_saved',remaining=len(remainder));delay(1)
  raise RuntimeError('RETRY_BUDGET_EXHAUSTED')
 def execute(label,poses,nav=False,cancel_distance=None,preempt_distance=None,fault_distance=None):
  nonlocal active,operation,action_id,current_handle,current_result,progress
  if sha(BASE/'plans/validated/manifest.json')!=binding:raise ValueError('PLAN_CHANGED')
  operation='FOLLOW';action_id=label;p=make_path(poses);pathpub.publish(p);g=NavigateToPose.Goal() if nav else FollowPath.Goal()
  if nav:g.pose=p.poses[-1]
  else:g.path=p;g.controller_id=a.controller;g.goal_checker_id='goal';g.progress_checker_id='progress'
  routes.append(dict(id=label,action='NavigateToPose' if nav else 'FollowPath',controller=a.controller,frame='map',poses=poses));dump(a.out/'routes.json',routes);active=True;log('goal_request',id=label,action=routes[-1]['action'],controller=a.controller,goal=poses[-1]);sf=(navigate if nav else follow).send_goal_async(g);wait(sf.done,5,20);current_handle=sf.result()
  if not current_handle.accepted:active=False;log('goal_rejected',id=label);return {'status':'REJECTED'}
  current_result=current_handle.get_result_async();start=st['t'];poly=np.array(poses);lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(poly[:,:2],axis=0),axis=1))];idx=0;checkpoint_idx=1;cancelled=False;last_save=-1;preempted=False;faulted=False;old_results=[];stop_reason=None;invalid_since=None
  log('accepted',id=label)
  while not current_result.done():
   tick();idx=max(idx,int(np.argmin(np.linalg.norm(poly[:,:2]-xy(),axis=1))));progress=dict(id=label,frame='map',index=idx,distance_m=float(lengths[idx]),remaining_poses=poses[min(idx,len(poses)-2):],t=st['t'],work_credited=False)
   if st['t']-last_save>.2:dump(a.out/'progress.json',progress);last_save=st['t']
   if a.case in ['inspect','diff_point_and_inspect'] and inspection_reference is not None:
    road_index=int(np.argmin(np.linalg.norm(inspection_reference[:,:2]-xy(),axis=1)))
    for arc in [2,4]:
     if inspection_distances[road_index]>=arc and arc not in seen_checkpoints:checkpoint('arc_'+str(arc)+'m');seen_checkpoints.add(arc)
   if fault_distance is not None and lengths[idx]>=fault_distance and not faulted:
    faulted=True;dump(a.out/'runtime_request.json',dict(id=label,operation='localization_loss',t=st['t']));log('localization_fault_request',id=label)
   if preempt_distance is not None and lengths[idx]>=preempt_distance and not preempted:
    preempted=True;old_results.append(current_result);g.path=make_path(poses[idx:]);log('preempt_request',id=label,remaining=len(poses)-idx);sf=follow.send_goal_async(g);wait(sf.done,5,20);current_handle=sf.result()
    if not current_handle.accepted:raise ValueError('PREEMPT_REJECTED')
    current_result=current_handle.get_result_async()
   if not nav and float(np.min(np.linalg.norm(poly[:,:2]-xy(),axis=1)))>.25 and not cancelled:
    log('path_departure',id=label);stop_reason='PATH_DEPARTURE';active=False;current_handle.cancel_goal_async();cancelled=True
   if cancel_distance is not None and lengths[idx]>=cancel_distance and not cancelled:
    log('cancel_request',id=label,distance_m=float(lengths[idx]));stop_reason='REQUESTED_CANCEL';active=False;current_handle.cancel_goal_async();cancelled=True
   # Gate stops immediately. Cancel the task only for persistent (>1.0 s)
   # observation loss, avoiding churn from one delayed perception frame.
   if admitted():invalid_since=None
   elif invalid_since is None:invalid_since=st['t']
   if invalid_since is not None and st['t']-invalid_since>1.0 and st['t']-start>3 and not cancelled:
    log('admission_lost',id=label,gate=st['gate'],frame_compatible=frame_ok(),owners_unique=owners_ok());active=False;current_handle.cancel_goal_async();cancelled=True
   if st['t']-start>120:log('action_timeout',id=label);active=False;current_handle.cancel_goal_async();wait(current_result.done,5,15);break
  rr=current_result.result();active=False
  for old in old_results:
   wait(old.done,3,12);log('preempted_result',status=old.result().status)
  log('action_result',id=label,status=rr.status,error_code=getattr(rr.result,'error_code',None),error_msg=getattr(rr.result,'error_msg',''),elapsed_s=st['t']-start);wait(stopped,3,12);delay(.5);log('stopped',id=label,v=st['odom'].twist.twist.linear.x,w=st['odom'].twist.twist.angular.z);return dict(status=rr.status,error_code=getattr(rr.result,'error_code',None),elapsed_s=st['t']-start,progress=progress,stop_reason=stop_reason,fault_requested=faulted,preempted=preempted)
 try:
  if a.case.startswith('calibration_') or a.case=='near_obstacle':
   wait(lambda:st['t']>=40,45,150);completed=True;log('stationary_calibration_observations_complete')
  else:
   wait(lambda:follow.server_is_ready() and navigate.server_is_ready(),60,180);wait(admitted,85,240);wait(audit_ready,10,40);wait(stopped,5,20);delay(2);log('admitted',global_status=st['global_'])
   # Survey geometry -> registered map frame; Nav2 transforms with the current TF.
   def convert(poses):return [[p[0]-28,p[1]-14,p[2]] for p in poses]
   if a.case in ['inspect','diff_point_and_inspect']:
    from connector12 import road_geometry
    from common12 import config
    poses=road_geometry(config())[0];dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(np.array(poses)[:,:2],axis=0),axis=1))];poses=[p for p,d in zip(poses,dist) if d<=6.]
   else:
    src=next(s for s in m['zones']['B']['plans'][0]['segments'] if s['kind']==('TURN' if a.vehicle=='ackermann' else 'COVERAGE'))['poses'];p0=src[0];angle=.15-p0[2];rot=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]]);poses=[[*(rot@(np.array(p[:2])-p0[:2])+[30.5,15.3]),p[2]+angle] for p in src];poses=[p for p in poses if math.dist(p[:2],poses[0][:2])<=2.]
    dump(a.out/'route_scope.json',dict(source_plan=m['plan_revision'],source='B1 first TURN (Ackermann) / COVERAGE (diff), anchor chord <=2 m; measured arc length in metrics',registration='declared commissioning anchor [30.5,15.3,0.15] in public prior frame',tool_on=False,whole_cluster_executed=False))
   dump(a.out/'route_prior.json',dict(frame='survey',poses=poses,source_plan=m['plan_revision'],tool_on=False));route=convert(poses)
   inspection_reference=np.array(route);inspection_distances=np.r_[0,np.cumsum(np.linalg.norm(np.diff(inspection_reference[:,:2],axis=0),axis=1))]
   if np.linalg.norm(xy()-np.array(route[0][:2]))>.30:raise ValueError('START_POSE_ADMISSION_FAILED')
   if a.vehicle=='ackermann':
    dh=(route[1][2]-route[0][2]+math.pi)%(2*math.pi)-math.pi;k=dh/max(.001,math.dist(route[0][:2],route[1][:2]));align(float(np.clip(k,-1/2.11,1/2.11)))
   if a.case=='diff_point_and_inspect':
    point_end=min(range(len(route)),key=lambda i:abs(math.dist(route[0][:2],route[i][:2])-.6));point=follow_resuming('service_transit',route[:point_end+1],nav=True)
    if point['status']!=4:raise ValueError('POINT_TRANSIT_FAILED')
    checkpoint('service_arrival');route=route[point_end:]
   outcome=follow_resuming('inspection' if a.case in ['inspect','diff_point_and_inspect'] else 'coverage_fragment',route)
   if outcome['status']!=4:raise ValueError('NAV_ACTION_DID_NOT_SUCCEED '+str(outcome['status']))
   if a.case in ['inspect','diff_point_and_inspect']:checkpoint('arc_6m_terminal')
   if a.case=='semantics':
    def extension(distance=1.2):
     q=st['pose'].pose.pose;start=xy();axis=np.array([math.cos(.15),math.sin(.15)]);direction=1 if math.cos(yaw(q.orientation)-.15)>0 else -1
     # Exercise action semantics inside the declared commissioning corridor.
     position=float(np.dot(start-np.array([2.5,1.3]),axis))
     if not 0<=position+direction*distance<=2.1:direction=-direction
     h=.15 if direction>0 else .15+math.pi
     return [[*(start+axis*direction*d),h] for d in np.linspace(0,distance,31)]
    for nav in [False,True]:
     rr=follow_resuming('cancel_'+('navigate' if nav else 'follow'),extension(),nav=nav,cancel_distance=.2)
     if rr['status']!=5:raise ValueError('CANCEL_NOT_CONFIRMED')
     tail=rr['progress']['remaining_poses'];dump(a.out/'canceled_residual.json',rr['progress']);log('cancel_verified',action='NavigateToPose' if nav else 'FollowPath',retained_poses=len(tail))
     rr=follow_resuming('resume_'+('navigate' if nav else 'follow'),tail,nav=nav)
     if rr['status']!=4:raise ValueError('RESUME_FAILED')
    old_controller=a.controller;a.controller='invalid_controller';bad=execute('invalid_controller',extension(.4));a.controller=old_controller
    if bad['status']!=6:raise ValueError('INVALID_CONTROLLER_ACCEPTED')
    rr=follow_resuming('preempt_follow',extension(.8),preempt_distance=.2)
    if rr['status']!=4:raise ValueError('PREEMPT_NEW_GOAL_FAILED')
    far=extension(.2);far[-1][0]+=1000.;rr=execute('unreachable_goal',far,nav=True)
    if rr['status']!=6:raise ValueError('UNREACHABLE_GOAL_NOT_ABORTED')
    a.controller='MPPI';rr=follow_resuming('controller_switch',extension(.5));a.controller=old_controller
    if rr['status']!=4:raise ValueError('CONTROLLER_SWITCH_FAILED')
    request='restart_after_cancel';dump(a.out/'runtime_request.json',dict(id=request,operation='restart_nav',t=st['t']));log('restart_requested');wait(lambda:(a.out/'runtime_reply.json').exists() and read(a.out/'runtime_reply.json').get('id')==request,30,120);follow.destroy();navigate.destroy();follow=ActionClient(n,FollowPath,'/follow_path');navigate=ActionClient(n,NavigateToPose,'/navigate_to_pose');await_nav_active();wait(lambda:follow.server_is_ready() and navigate.server_is_ready(),20,80);rr=follow_resuming('after_restart',extension(.5),nav=True)
    if rr['status']!=4:raise ValueError('POST_RESTART_ACTION_FAILED')
   if a.case=='localization_loss':
    q=st['pose'].pose.pose;h=yaw(q.orientation)+math.pi;start=xy();r=[[*(start+np.array([math.cos(h),math.sin(h)])*d),h] for d in np.linspace(0,1.5,31)]
    for attempt in range(6):
     revalidate();rr=execute('fault_navigation_'+str(attempt),r,fault_distance=.25)
     if rr['fault_requested']:break
     if rr['status']!=5:raise ValueError('FAULT_NOT_INJECTED')
     r=rr['progress']['remaining_poses'];delay(1.)
    if not rr['fault_requested']:raise ValueError('FAULT_INJECTION_RETRY_EXHAUSTED')
    if rr['status']!=5:raise ValueError('LOCALIZATION_LOSS_NOT_CANCELED')
    dump(a.out/'fault_residual.json',rr['progress']);revalidate();rr=follow_resuming('after_localization_restore',rr['progress']['remaining_poses'])
    if rr['status']!=4:raise ValueError('LOCALIZATION_RESUME_FAILED')
   if a.case=='ackermann_follow_path':
    align(0.);log('stationary_steering_verified');q=st['pose'].pose.pose;h=yaw(q.orientation);start=xy();r=[[*(start+np.array([math.cos(h),math.sin(h)])*d),h] for d in np.linspace(0,1.,26)];rr=follow_resuming('cancel_ackermann',r,cancel_distance=.3)
    if rr['status']!=5:raise ValueError('ACK_CANCEL_NOT_CONFIRMED')
   completed=True;delay(2)
 except BaseException as e:log('error',error=repr(e));print(repr(e),flush=True)
 finally:
  active=False
  if current_result is not None and not current_result.done() and current_handle:current_handle.cancel_goal_async()
  deadline=time.monotonic()+3.;confirmed_since=None;last_stop=-1e9
  while rclpy.ok() and time.monotonic()<deadline:
   if time.monotonic()-last_stop>.03:
    last_stop=time.monotonic();lease.publish(String(data=json.dumps(dict(t=st['t'],active=False,operation='STOP',action_id=action_id,plan_revision=m['plan_revision']))))
   rclpy.spin_once(n,timeout_sec=.01)
   if stopped() and (st['tool'] or {}).get('stopped_confirmed'):
    if confirmed_since is None:confirmed_since=time.monotonic()
    if time.monotonic()-confirmed_since>.4:break
   else:confirmed_since=None
  dump(a.out/'shutdown.json',dict(state='STOPPED' if stopped() and (st['tool'] or {}).get('stopped_confirmed') else 'UNCONFIRMED',tool_on_requested=False));dump(a.out/'result.json',dict(t=st['t'],completed=completed,events=events,controller=a.controller,case=a.case));f.close();n.destroy_node();rclpy.try_shutdown()
 return 0 if completed else 1
if __name__=='__main__':raise SystemExit(main())
