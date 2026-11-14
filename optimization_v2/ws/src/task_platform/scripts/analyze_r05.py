from common import *
import math,numpy as np
from analyze_r02 import rows
from scipy.spatial.transform import Rotation

def analyze(out):
 runtime=read(out/'runtime.json');spec=read(out/'spec.json');checks={'runtime_clean':runtime['passed'],'original_map_unchanged':runtime['original_db_unchanged']};result={}
 if not (out/'observer_result.json').exists():return dict(passed=False,checks=checks,error='NO_OBSERVER_RESULT')
 observer=read(out/'observer_result.json');observations=rows(out/'observations.jsonl');truth=rows(out/'evaluation/truth.jsonl');t=np.array([r['t'] for r in truth]);xy=np.array([r['xyz'][:2] for r in truth]);yaw=np.unwrap([r['rpy'][2] for r in truth]);valid=[r for r in observations if r['kind']=='quality' and r.get('valid')];first=[r for r in valid if r['t']<65];after=[r for r in valid if r['t']>75]
 checks['completed']=observer['complete'];checks['all_sensors']=all(observer['counts'].get(k,0)>50 for k in ['lidar','camera','imu','gnss','odom','rtab_icp_info']);checks['observed_map_match_before_and_after_restart']=len(first)>=5 and len(after)>=5 and observer['restarted'];checks['initial_acquisition_within_45s']=bool(first) and first[0]['t']<=45
 restart=read(out/'restart_done.json') if (out/'restart_done.json').exists() else {};checks['restart_acquisition_within_45s']=bool(after) and 0<after[0]['t']-restart.get('t',1e6)<=45
 cloud=[r for r in observations if r['kind']=='lidar'];camera=[r for r in observations if r['kind']=='camera'];checks['native_3d_raster']=bool(cloud) and all(r['height']==read(out/'scene/sensor_manifest.json')['lidar']['vertical'] and r['width']==read(out/'scene/sensor_manifest.json')['lidar']['horizontal'] for r in cloud);checks['camera_nonblank']=bool(camera) and min(r['std'] for r in camera)>5
 errors=[]
 for r in valid:
  qt=r['t'];truthxy=np.array([np.interp(qt,t,xy[:,i]) for i in range(2)]);estimate=np.array(r['pose'][:2])+[28,14];theta=Rotation.from_quat(r['pose'][3:]).as_euler('xyz')[2];diff=math.atan2(math.sin(theta-np.interp(qt,t,yaw)),math.cos(theta-np.interp(qt,t,yaw)));errors.append([qt,float(np.linalg.norm(estimate-truthxy)),diff])
 errors=np.array(errors);global_xy=float(np.sqrt(np.mean(errors[:,1]**2))) if len(errors) else None;global_yaw=float(np.degrees(np.sqrt(np.mean(errors[:,2]**2)))) if len(errors) else None;checks['registered_map_accuracy']=bool(len(errors)) and global_xy<=.15 and global_yaw<=3
 odom=[r for r in observations if r['kind']=='odom' and r['t']>6];start=read(out/'scene/fixture.json')['spawn'];R=Rotation.from_euler('z',start[2]).as_matrix()[:2,:2];local=[]
 for r in odom:
  qt=r['t'];ground=np.array([np.interp(qt,t,xy[:,i]) for i in range(2)]);est=R@np.array(r['pose'][:2])+start[:2];local.append([qt,float(np.linalg.norm(est-ground))])
 local=np.array(local);local_rmse=float(np.sqrt(np.mean(local[:,1]**2))) if len(local) else None;checks['local_odom_rmse_under_25cm']=local_rmse is not None and local_rmse<.25
 distance=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum());checks['observed_motion_over_3m']=observer['moved'] and distance>3.;last=observer['feedback'];checks['final_stopped']=last is not None and not last['armed'] and abs(last['limited_v'])<.001
 # A single public odom transform owner is checked through graph endpoint GIDs too.
 checks['odom_transform_present']=observer['tf_edges'].get('odom->base_link',0)>50
 checks['sole_public_tf_owner']=len(observer.get('tf_publishers',[]))==1 and observer['tf_publishers'][0]['node']=='s05_local_odometry'
 checks['probe_source_binding']=spec.get('online_probe_sha256')==sha(PKG/'scripts/observer_r05.py')
 checks['runtime_source_binding']=spec.get('runtime_sha256')==sha(PKG/'scripts/runtime_r05.py')
 binding=read(edition('R05')/spec['task']/'binding.json');checks['current_rig_binding']=spec['rig_revision']==binding['rig_revision'] and spec['binding_revision']==binding['revision']
 adaptation=read(out/'overlay/adaptation.json');checks['generated_adapters_intact']=all(sha(out/'overlay'/name)==digest for name,digest in adaptation['generated'].items()) and all(sha(ROOT/name)==digest for name,digest in adaptation['upstream'].items())
 result=dict(passed=all(checks.values()),task=spec['task'],checks={k:bool(v) for k,v in checks.items()},valid_map_matches=[len(first),len(after)],first_acquisition_s=first[0]['t'] if first else None,reacquisition_s=after[0]['t']-restart.get('t',0) if after else None,map_xy_rmse_m=global_xy,map_yaw_rmse_deg=global_yaw,local_xy_rmse_m=local_rmse,actual_travel_m=distance,sensor_counts=observer['counts'],sim_seconds=observer['t'],wall_seconds=runtime['wall_seconds'],new_rig_revision=spec['rig_revision'],truth_evaluation_only=True,scope='Existing registered frozen-cloud localization + native RTAB ICP + wheel/IMU, independent GNSS gate, actual global-localizer restart; work task not accepted')
 dump(out/'analysis_r05.json',result);np.savez(out/'errors.npz',global_error=errors,local_error=local);return result

def figures(paths):
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,axs=plt.subplots(3,1,figsize=(12,8),sharex=True)
 for ax,(task,p) in zip(axs,paths.items()):
  d=np.load(p/'errors.npz');g=d['global_error'];l=d['local_error']
  if len(g):ax.plot(g[:,0],g[:,1],'.',ms=2,label='frozen map match XY')
  if len(l):ax.plot(l[:,0],l[:,1],lw=.8,label='local odometry XY')
  ax.axvline(65,ls='--',c='red',label='global localizer restart request');ax.grid(alpha=.3);ax.set(title=task,ylabel='Error [m]');ax.legend()
 axs[-1].set_xlabel('Simulation seconds');fig.tight_layout();p=APP/'results/R05/localization.png';fig.savefig(p,dpi=150);plt.close(fig);return [p]
