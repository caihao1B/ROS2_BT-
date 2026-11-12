"""Independent evaluator. May read domain 120 truth only AFTER the runtime ends."""
from common13 import *
import re
def percentile(x,p):return float(np.percentile(x,p)) if len(x) else None
def angle(x):return (x+np.pi)%(2*np.pi)-np.pi

def tracking(xy,heading,reference):
 q=np.asarray(reference);a=q[:-1,:2];v=np.diff(q[:,:2],axis=0);l=np.sum(v*v,axis=1);l[l<1e-12]=1e-12
 u=np.clip(np.sum((xy[:,None,:]-a[None,:,:])*v[None,:,:],axis=2)/l,0,1);proj=a[None,:,:]+u[:,:,None]*v[None,:,:];dist=np.linalg.norm(xy[:,None,:]-proj,axis=2);i=np.argmin(dist,axis=1);idx=np.arange(len(xy));h=q[i,2]+u[idx,i]*angle(q[i+1,2]-q[i,2]);return dist[idx,i],np.abs(angle(heading-h))

def evaluate(out):
 out=Path(out);runtime=read(out/'runtime.json');client=read(out/'client/result.json') if (out/'client/result.json').exists() else {};obs=rows(out/'online/observations.jsonl');events=client.get('events',[]);gate=rows(out/'gate.jsonl');truth=rows(out/'evaluation/truth.jsonl');metrics=[d for d in obs if d['record_type']=='metrics'];audit=read(out/'online/result.json') if (out/'online/result.json').exists() else {};enc=[d for d in obs if d['record_type']=='encoders'];resources=read(out/'resources.json');checks={};r=dict(vehicle=runtime['vehicle'],controller=client.get('controller'),case=runtime['case'],completed=bool(runtime['success'] and client.get('completed')),runtime_error=runtime.get('error'),sim_s=client.get('t',0),wall_s=runtime['wall_seconds'],database_unchanged=runtime['database_unchanged'])
 r['max_rss_mib']=max([x['owned_rss_bytes']/1048576 for x in resources] or [0]);r['peak_cpu_cores']=max([max(0,(b.get('owned_cpu_seconds',0)-a.get('owned_cpu_seconds',0))/(b['wall']-a['wall'])) for a,b in zip(resources,resources[1:])] or [0]);r['compute_p95_ms']=percentile([x['compute_ms'] for x in metrics],95);r['compute_samples']=len(metrics);r['compute_p95_target_met']=bool(r['compute_p95_ms'] is not None and r['compute_p95_ms']<40)
 times=np.array([x['t'] for x in metrics]);dt=np.diff(times);dt=dt[(dt>0)&(dt<.5)];r['median_active_compute_hz_sim']=1/float(np.median(dt)) if len(dt) else None;r['compute_period_p95_sim_s']=percentile(dt,95);r['effective_compute_hz_sim']=float(len(dt)/sum(dt)) if len(dt) else None
 r['command_limits_ok']=bool(gate) and all(-1e-8<=d['v']<=.12000001 and abs(d['w'])<=.35000001 and (r['vehicle']=='diff' or abs(d['w'])<=d['v']/2.11+1e-8) for d in gate)
 r['fault_injection_verified']=all(any(e['event']==name for e in runtime.get('events',[])) for name in ['source_observations_paused','source_observations_resumed'])
 floor=next((e.get('startup_clock_floor') for e in runtime.get('events',[]) if e['event']=='nav_processes_restarted'),None);clock_records={}
 for role in ['controller','planner']:
  log=out/(role+'_restarted.log')
  clock_records[role]=[list(map(float,values)) for values in re.findall(r'STARTUP_CLOCK_READY floor=([0-9.]+) node=([0-9.]+) costmap=([0-9.]+)',log.read_text() if log.exists() else '')]
 r['restart_clock_records']=clock_records;r['restart_clock_ready']=bool(floor is not None and floor>0 and all(records and all(abs(v[0]-floor)<.001 and v[1]>=v[0] and v[2]>=v[0] and abs(v[1]-v[2])<.1 for v in records) for records in clock_records.values()))
 r['nav_restart_verified']=r['restart_clock_ready'] and len([c for c in runtime.get('commands',[]) if c['role'] in ['controller_restarted','planner_restarted','navigator_restarted','nav_lifecycle_restarted']])==4
 active_audits=[d for d in audit.get('audits',[]) if d.get('navigation_active',False) or any(e['event']=='goal_request' and 0<=d['t']-e['t']<1 for e in events)]
 r['ownership_ok']=bool(active_audits) and all(d['request']==['s03_test_driver'] and d['actuator']==['s03_drive_bridge'] and d['nav']==['controller_server'] and d['lease']==['s13_action_client'] and sorted(d['tf'])==['s05_local_odometry','s08_global_pose_manager'] for d in active_audits)
 subscriptions=audit.get('subscriptions',{});r['truth_isolated']=not any(v for k,v in subscriptions.items() if '/evaluation/' in k or '/truth' in k)
 r['restart_handshake_verified']=any(e['event']=='restart_discovery_ready' for e in events)
 r['actions']=[e for e in events if e['event'] in ['action_result','preempted_result','cancel_request','cancel_verified','align_complete','nav_lifecycle_active_verified']];r['resumes']=sum(e['event']=='resume_saved' for e in events);r['checkpoints']=[e for e in events if e['event']=='checkpoint'];r['checkpoint_observations_current']=bool(r['checkpoints']) and all(e['observations_current'] for e in r['checkpoints'])
 contacts=read(out/'evaluation/contacts.json') if (out/'evaluation/contacts.json').exists() else {};r['contacts']=contacts;r['obstacle_contacts']=sum(v for k,v in contacts.items() if 'ground::body::ground' not in k)
 # Physical trajectory versus declared original survey route, never estimated output as truth.
 ref=read(out/'client/route_prior.json')['poses'] if (out/'client/route_prior.json').exists() else None
 r['reference_length_m']=float(np.linalg.norm(np.diff(np.array(ref)[:,:2],axis=0),axis=1).sum()) if ref else None
 active_start=next((e['t'] for e in events if e['event']=='goal_request'),float('inf'));base_end=max([e['t'] for e in events if e['event']=='action_result' and (e['id'].startswith('coverage_fragment') or e['id'].startswith('inspection') or e['id']=='service_transit')] or [0]);selected=[d for d in truth if active_start<=d['t']<=base_end]
 if ref and len(selected)>1:
  xy=np.array([d['xyz'][:2] for d in selected]);h=np.array([d['rpy'][2] for d in selected]);error,he=tracking(xy,h,ref);r.update(tracking_p95_m=percentile(error,95),tracking_max_m=float(max(error)),heading_max_rad=float(max(he)),tracking_samples=len(selected),terminal_error_m=float(np.linalg.norm(xy[-1]-np.array(ref[-1][:2]))));dump(out/'tracking.json',dict(t=[d['t'] for d in selected],actual=xy.tolist(),reference=ref,error_m=error.tolist(),heading_error_rad=he.tolist()))
 else:r.update(tracking_p95_m=None,tracking_max_m=None,heading_max_rad=None,tracking_samples=0,terminal_error_m=None)
 r['tracking_thresholds_ok']=r['tracking_max_m'] is not None and r['tracking_max_m']<=.30 and r['heading_max_rad']<=.15
 stops=[]
 if len(truth)>8:
  t=np.array([x['t'] for x in truth]);xy=np.array([x['xyz'][:2] for x in truth]);h=np.unwrap([x['rpy'][2] for x in truth]);v=np.linalg.norm(xy[5:]-xy[:-5],axis=1)/(t[5:]-t[:-5]);w=np.abs(h[5:]-h[:-5])/(t[5:]-t[:-5]);ts=t[5:]
  gate_stops=[dict(t=b['t'],event='gate_stop',id=b['action_id'],reason=b['reason']) for a,b in zip(gate,gate[1:]) if b['mode']=='STOP' and (abs(a['v'])>.005 or abs(a['w'])>.01)]
  for e in events+gate_stops:
   if e['event'] not in ['cancel_request','admission_lost','action_result','gate_stop']:continue
   i=np.where((ts>=e['t'])&(v<.01)&(w<.02))[0];delay=float(ts[i[0]]-e['t']) if len(i) else None;stops.append(dict(t=e['t'],event=e['event'],id=e.get('id'),physical_stop_delay_s=delay))
 r['stops']=stops;r['max_stop_delay_s']=max([d['physical_stop_delay_s'] for d in stops if d['physical_stop_delay_s'] is not None] or [0]);r['stop_thresholds_ok']=bool(stops) and all(d['physical_stop_delay_s'] is not None and d['physical_stop_delay_s']<=2 for d in stops)
 r['shutdown_confirmed']=runtime.get('tool_shutdown',{}).get('state')=='STOPPED';r['passed']=r['obstacle_contacts']==0 and all(r[k] for k in ['completed','database_unchanged','command_limits_ok','ownership_ok','truth_isolated','tracking_thresholds_ok','stop_thresholds_ok','shutdown_confirmed']);dump(out/'metrics.json',r);return r
if __name__=='__main__':print(json.dumps(evaluate(Path(sys.argv[1])),ensure_ascii=False,indent=2))
