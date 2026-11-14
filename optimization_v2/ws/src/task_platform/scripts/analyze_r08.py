from common import *
from analyze_r02 import rows
import numpy as np,math
from shapely.geometry import LineString,Point,Polygon,shape
from shapely.affinity import translate,rotate

def analyze(out):
 runtime=read(out/'runtime.json');spec=read(out/'spec.json');cfg=read(out/'mission.json');mission=read(out/'mission_result.json') if (out/'mission_result.json').exists() else {};case=spec['case'];checks=dict(runtime_clean=runtime['passed'],original_database_unchanged=runtime['original_db_unchanged']);truth=rows(out/'evaluation/truth.jsonl');xy=np.array([x['xyz'][:2] for x in truth]);times=np.array([x['t'] for x in truth]);line=LineString(np.array(cfg['route_world'])[:,:2]);cross=[line.distance(Point(p)) for p in xy];progress=[line.project(Point(p)) for p in xy];refs=rows(out/'reference/reference.jsonl');errors=[]
 for row in refs:
  if row.get('valid') and 'pose' in row:errors.append(math.dist(np.asarray(row['pose'][:2])+[28,14],[np.interp(row['odom_t'],times,xy[:,j]) for j in range(2)]))
 rmse=float(np.sqrt(np.mean(np.square(errors)))) if errors else None;contacts=read(out/'evaluation/contacts.json');bad={k:v for k,v in contacts.items() if 'vehicle::' in k and not ('wheel::' in k and 'ground::body::ground' in k)};fp=Polygon(cfg['vehicle']['footprint']);allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m']);outside=[]
 for row in truth[::5]:
  body=translate(rotate(fp,row['rpy'][2],use_radians=True),*row['xyz'][:2])
  if not allowed.covers(body):outside.append(row['t'])
 checks.update(no_unexpected_contact=not bad,map_rmse_under_15cm=rmse is not None and rmse<.15,physical_footprint_inside_authorized_area=not outside,stopped=bool(mission.get('final_gate',{}).get('stopped')))
 events=rows(out/'mission_events.jsonl')
 from tracking_error_r08 import errors as active_errors
 active_cross,references_authorized=active_errors(truth,events,np.array(cfg['route_world'])[:,:2])
 if case not in ['blocked']:checks.update(active_path_cross_track_under_50cm=bool(active_cross) and max(active_cross)<.5,executed_references_authorized=references_authorized)
 if case=='blocked':checks.update(expected_controlled_hold=mission.get('controlled_hold',False),residual_preserved=mission.get('remaining_original_m',0)>0,bounded_attempts=1<=mission.get('replans',0)<=2,no_false_completion=not mission.get('passed',False))
 else:checks.update(selected_route_completed=mission.get('passed',False),measured_progress=bool(progress) and max(progress)>=mission.get('selected_length_m',1e6)-.25)
 owners=mission.get('ownership',{});checks.update(unique_actuator=owners.get('/s03/actuator/request')==['r08_actuator_gate'],no_tool_owner=owners.get('/s10/tool/command')==[],unique_tf=set(owners.get('/tf',[]))=={'s05_local_odometry','s08_global_pose_manager'} and len(owners.get('/tf',[]))==2,source_binding=all((ROOT/p).is_file() and sha(ROOT/p)==h for p,h in spec['implementation_bindings'].items()),task_version=cfg['version']==read(edition('R08')/'mission.json')['version'])
 decisions=rows(out/'decisions.jsonl');events=rows(out/'mission_events.jsonl');tracks=rows(out/'tracking/tracks.jsonl');follow=[x for x in decisions if x.get('action')=='FOLLOW'];gaps=[x['evidence']['clearance_m'] for x in follow if 'clearance_m' in x.get('evidence',{})];actors=rows(out/'evaluation/dynamic.jsonl') if (out/'evaluation/dynamic.jsonl').exists() else [];physical=[];stop_test={}
 if actors:
  at=np.array([x['t'] for x in actors]);axy=np.array([x['xyz'][:2] for x in actors]);ay=np.unwrap([x.get('yaw',0.) for x in actors]);dims=[.5,7.2] if case=='blocked' else [.5,.5];box=Polygon([[-dims[0]/2,-dims[1]/2],[dims[0]/2,-dims[1]/2],[dims[0]/2,dims[1]/2],[-dims[0]/2,dims[1]/2]])
  for d in follow:
   t=d['t'];i=int(np.argmin(abs(times-t)));actor=translate(rotate(box,float(np.interp(t,at,ay)),use_radians=True),*[float(np.interp(t,at,axy[:,j])) for j in range(2)]);body=translate(rotate(fp,truth[i]['rpy'][2],use_radians=True),*xy[i]);physical.append(body.distance(actor))
  if case=='leader':
   # Fixture schedule is an assessment window only; actual measured positions
   # establish that the leader stopped and the robot reacted. Never online input.
   def motion(t0,t1,p,t):return float(np.linalg.norm([np.interp(t1,t,p[:,j])-np.interp(t0,t,p[:,j]) for j in range(2)])) if t[-1]>=t1 else None
   leader_stop=motion(62,67,axy,at);ego_stop=motion(64,67,xy,times);ego_resume=motion(72,85,xy,times);stop_test=dict(leader_motion_62_67_m=leader_stop,ego_motion_64_67_m=ego_stop,ego_motion_72_85_m=ego_resume)
   checks.update(measured_leader_stop=leader_stop is not None and leader_stop<.05,measured_follower_stop=ego_stop is not None and ego_stop<.05,measured_resume=ego_resume is not None and ego_resume>.5)
 if case=='leader':checks.update(observed_follow=bool(gaps) and min(gaps)>=2.,physical_follow_clearance=bool(physical) and min(physical)>=2.,stop_resume=any(x['event']=='BT_INTERRUPTS_PATH' for x in events) and any(x['event']=='RESUME_REMAINING_ROUTE' for x in events),tracking_no_fixture=bool(tracks) and all(not x.get('truth_used',True) for x in tracks))
 if case=='detour':checks.update(observed_patch_admitted=any(x['event']=='OBSERVED_PATCH_ADMITTED' for x in events),rejoin_confirmed=any(x['event']=='PATCH_REJOIN_CONFIRMED' for x in events),actual_lateral_detour=bool(cross) and max(cross)>.5)
 result=dict(passed=all(checks.values()),checks=checks,case=case,task_result=mission,map_xy_rmse_m=rmse,maximum_cross_track_m=max(cross) if cross else None,maximum_active_path_cross_track_m=max(active_cross) if active_cross else None,maximum_actual_progress_m=max(progress) if progress else 0,follow_decisions=len(follow),minimum_observed_follow_gap_m=min(gaps) if gaps else None,minimum_physical_follow_gap_m=min(physical) if physical else None,leader_stop_assessment=stop_test,unexpected_contacts=bad,full_connector_passed=bool(mission.get('full_route_selected') and all(checks.values())),formal_credit=False,scope='Declared independent inspection case only; blocked-case PASS means controlled hold, never route completion');dump(out/'analysis_r08.json',result);np.savez(out/'trajectory.npz',xy=xy,t=times);return result
