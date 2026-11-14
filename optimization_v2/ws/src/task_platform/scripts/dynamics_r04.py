"""Flat-ground speed qualification using independent holdout command profiles."""
from common import *
import numpy as np,math,copy
from probe_r02 import run
from analyze_r02 import rows,fit_circle

def build():
 require('R03');out=edition('R04')
 for task in TASKS:
  c=read(edition('R02')/'vehicles'/f'{task}.json');original=c['config_hash'];p=ROOT/read(APP/'results/R02'/(task+'_latest.json'))['path'];r=read(p/'analysis.json');ratio=1.
  if c['kind']=='differential':
   measured=np.mean([abs(r['phases'][k]['yaw_rate_rad_s']) for k in ['left','right']]);ratio=.32/float(measured);T=c['wheel']['track']*ratio;c['calibration'].update(effective_track_m=T,drive_track_m=T,scope='Gazebo flat contact calibration only; R02 .4 m/s, curvature .8; independently validated at R04 .6 m/s curvature .45')
  c['revision']='r04.1';c['upstream_vehicle_hash']=original;c['calibration']['gain_from_R02']=ratio;c['calibration']['source_analysis_sha256']=sha(p/'analysis.json');c['config_hash']=canonical({k:v for k,v in c.items() if k!='config_hash'});dump(out/'vehicles'/f'{task}.json',c)
 return out

def phases(c):
 ps=[dict(name='settle',mode='STOP',duration=2.,v=0.,w=0.)]
 def add(name,v,w,duration):
  ps.extend([dict(name='arm_'+name,mode='ARM',duration=.7,v=0.,w=0.),dict(name=name,mode='DRIVE',duration=duration,v=v,w=w),dict(name='stop_'+name,mode='STOP',duration=3.,v=0.,w=0.)])
 for v in [.3,.6,.8]:add('speed_'+str(v),v,0.,7.)
 k=.25 if c['kind']=='ackermann' else .45
 add('left_holdout',.6,.6*k,20.);add('right_holdout',.6,-.6*k,20.)
 add('reverse_curve',-.25,-.25*(.25 if c['kind']=='ackermann' else .45),18.)
 add('reverse_straight',-.25,0.,6.)
 ps.extend([dict(name='arm_loss',mode='ARM',duration=.7,v=0.,w=0.),dict(name='before_loss',mode='DRIVE',duration=6.,v=.8,w=0.),dict(name='command_loss',mode='DRIVE',duration=4.,v=.8,w=0.,send=False),dict(name='latched_reject',mode='DRIVE',duration=3.,v=.5,w=0.),dict(name='rearm',mode='ARM',duration=.7,v=0.,w=0.),dict(name='resume',mode='DRIVE',duration=5.,v=.3,w=0.),dict(name='final_stop',mode='STOP',duration=3.,v=0.,w=0.)])
 return ps

def evaluate(out):
 runtime=read(out/'runtime.json');c=read(out/'scene/vehicle.json');spec=read(out/'spec.json');truth=rows(out/'evaluation/truth.jsonl');online=rows(out/'online.jsonl');t=np.array([r['t'] for r in truth]);xy=np.array([r['xyz'][:2] for r in truth]);yaw=np.unwrap([r['rpy'][2] for r in truth]);dt=np.diff(t);tt=(t[:-1]+t[1:])/2;delta=np.diff(xy,axis=0);speed=np.linalg.norm(delta,axis=1)/dt;angular=np.diff(yaw)/dt;projection=(delta*np.c_[np.cos(yaw[:-1]),np.sin(yaw[:-1])]).sum(axis=1)/dt
 phases_result={};checks={'runtime_clean':runtime['passed'],'completed':read(out/'driver_result.json')['complete']};cursor=0.;prev=None;brakes=[]
 for ph in spec['phases']:
  a=cursor;b=a+ph['duration'];cursor=b;name=ph['name'];idx=(tt>a+2)&(tt<b-.2);feedback=[x for x in online if a+2<x['t']<b-.2];d=dict(start=a,end=b)
  if ph['mode']=='DRIVE' and ph.get('send',True) and name!='latched_reject':
   d.update(signed_speed=float(np.median(projection[idx])),yaw_rate=float(np.median(angular[idx])));checks[name+'_speed']=abs(d['signed_speed']/ph['v']-1)<.12
   if ph['w']:
    points=xy[(t>a+2)&(t<b-.2)];d.update(fit_circle(points));d['expected_radius']=abs(ph['v']/ph['w']);d['relative_radius_error']=abs(d['radius_m']/d['expected_radius']-1);checks[name+'_turn']=d['relative_radius_error']<.08 and d['radial_rmse_m']<.08 and np.sign(d['yaw_rate'])==np.sign(ph['w'])
   samples=[x for x in online if a<x['t']<b];et=np.array([x['encoders']['t'] for x in samples]);ev=np.array([x['encoders']['limited_v'] for x in samples]);valid=np.diff(et)>1e-6;acc=np.diff(ev)[valid]/np.diff(et)[valid]
   d['limited_acceleration_abs_p99']=float(np.quantile(np.abs(acc),.99)) if len(acc) else None;checks[name+'_acceleration']=len(acc)>5 and max(np.abs(acc))<=c['limits']['acceleration']+.01
  if (ph['mode']=='STOP' and prev and prev['mode']=='DRIVE') or name=='command_loss':
   cand=np.flatnonzero((tt>=a)&(tt<=b));stops=[i for i in cand if speed[i]<c['safety']['stopped_v_mps'] and all(speed[j]<.012 for j in cand if tt[i]<=tt[j]<=tt[i]+.25)]
   end=tt[stops[0]] if stops else b;ix=(tt>=a)&(tt<=end);distance=float(np.sum(speed[ix]*dt[ix]));d.update(stop_delay_s=float(end-a),stop_distance_m=distance);limit=abs(prev['v'])**2/(2*c['limits']['deceleration'])+(.35*abs(prev['v']) if name=='command_loss' else 0)+.10
   d['distance_budget_m']=limit;checks[name+'_brake']=bool(stops) and d['stop_delay_s']<=c['safety']['stop_delay_s'] and distance<=limit;brakes.append(dict(phase=name,**d))
  if name in ['command_loss','latched_reject']:
   checks[name+'_latched']=len(feedback)>5 and all(not x['encoders']['armed'] and x['encoders']['reason']=='ACTUATOR_WATCHDOG' for x in feedback) and max(speed[idx],default=math.inf)<.01
  phases_result[name]=d;prev=ph
 last=online[-1]['encoders'];checks['final_stopped']=not last['armed'] and abs(last['limited_v'])<.001 and max(abs(x*c['wheel']['radius']) for x in last['velocity'][:2])<.01
 result=dict(passed=all(checks.values()),task=c['task_id'],vehicle_hash=c['config_hash'],checks={k:bool(v) for k,v in checks.items()},phases=phases_result,braking=brakes,sim_seconds=float(t[-1]),wall_seconds=runtime['wall_seconds'],truth_evaluation_only=True,scope='flat rigid contact dynamics; no terrain, localization, obstacle or hardware certification');dump(out/'analysis_r04.json',result);return result

def figures(runs):
 import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
 fig,axs=plt.subplots(3,1,figsize=(13,9),sharex=True)
 for ax,(task,out) in zip(axs, runs.items()):
  tr=rows(out/'evaluation/truth.jsonl');on=rows(out/'online.jsonl');t=np.array([x['t'] for x in tr]);xy=np.array([x['xyz'][:2] for x in tr]);yaw=np.array([x['rpy'][2] for x in tr]);v=(np.diff(xy,axis=0)*np.c_[np.cos(yaw[:-1]),np.sin(yaw[:-1])]).sum(axis=1)/np.diff(t)
  ax.plot(t[:-1],v,label='measured body speed',lw=1.5);ax.plot([x['t'] for x in on],[x['request']['v'] if x['phase']!='command_loss' else float('nan') for x in on],label='requested',alpha=.6);ax.set(ylabel='m/s',title=task);ax.grid(alpha=.3);ax.legend(loc='upper right')
 axs[-1].set_xlabel('Simulation seconds');fig.tight_layout();p=APP/'results/R04/dynamics.png';fig.savefig(p,dpi=150);plt.close(fig);return [p]
