"""Independent scoring domain120 output only. No trajectory alignment or truth online."""
from common08 import *
from mapping_analysis import rows
import importlib.util

def analyze(out):
 out=Path(out);rt=read(out/'runtime.json');end=read(out/'online/result.json');obs=rows(out/'online/observations.jsonl');truth=rows(out/'evaluation/truth.jsonl');commands=rows(out/'motion/commands.jsonl');status=[r for r in obs if r['key']=='status'];poses=[r for r in obs if r['key']=='pose'];gps=[r for r in obs if r['key']=='gnss'];maps=[r for r in obs if r['key']=='map'];tt=np.array([r['t'] for r in truth]);xyz=np.array([r['xyz'] for r in truth]);yaw=np.unwrap([r['rpy'][2] for r in truth]);st=np.array([r['t'] for r in status]);trajectory=[]
 for row in poses:
  t=row['t'];p=row['pose'];reference=np.array([np.interp(t,tt,xyz[:,i]) for i in range(3)])-[28,14,0];s=status[max(0,np.searchsorted(st,t,side='right')-1)];h=Rotation.from_quat(p[3:]).as_euler('xyz')[2];delta=(h-np.interp(t,tt,yaw)+np.pi)%(2*np.pi)-np.pi
  trajectory.append(dict(t=t,estimate=p[:3],truth=reference.tolist(),xy_error_m=float(np.linalg.norm(np.array(p[:2])-reference[:2])),yaw_error_rad=float(abs(delta)),allowed=s['navigation_allowed'],state=s['state']))
 def quantile(arr,key):return float(np.quantile([r[key] for r in arr],.95)) if arr else None
 qualified=[r for r in trajectory if r['allowed']];first=next((s['t'] for s in status if s['navigation_allowed']),None);intervals=[(max(0,b['t']-max(first,a['t'])),a['navigation_allowed']) for a,b in zip(status,status[1:]) if first is not None and b['t']>=first];active=sum(dt for dt,allowed in intervals if allowed);total=sum(dt for dt,_ in intervals)
 checks=dict(completed=rt['success'] and end['completed'],frozen_database_unchanged=rt['database_unchanged'],map_bundle_valid=bool(verify_bundle()),no_unadmitted_drive=all(c['status'] and c['status']['navigation_allowed'] and c['v']<=c['status']['max_speed'] for c in commands if c['mode']=='DRIVE'),unique_tf_and_actuator=bool(end['audits']) and all(sorted(r['tf'])==['s05_local_odometry','s08_global_pose_manager'] and r['actuator']==['s03_drive_bridge'] for r in end['audits'] if r['t']>10),truth_isolation=not any('/evaluation/' in topic or '/s04/raw/' in topic or topic.startswith('/world/') for topic in end['subscriptions']))
 checks['tf_edges']={(r['parent'],r['child']) for r in end['tf_edges']}=={('map','odom'),('odom','base_link')}
 checks['no_global_feedback']=all(not any(name in ['s05_fused','s05_wheel_imu','s07_slam'] for name in end['subscriptions'].get(topic,[])) for topic in ['/s08/gnss','/s08/map','/s08/pose','/s07/localization/evidence'])
 from runtime import s08_inputs
 from s07_common import capture_inputs
 checks['capture_versions_match']=read(out/'s08_inputs.json')==s08_inputs() and read(out/'capture_inputs.json')==capture_inputs()
 contacts=read(out/'evaluation/contacts.json');checks['no_obstacle_contact']=not any('vehicle::' in k and not any(x in k for x in ['ground::','grade_']) for k in contacts)
 m=dict(case=rt['case'],pose_name=rt['pose_name'],checks=checks,first_admission_s=first,xy_p95_m=quantile(qualified,'xy_error_m'),yaw_p95_rad=quantile(qualified,'yaw_error_rad'),all_published_xy_p95_m=quantile(trajectory,'xy_error_m'),all_published_yaw_p95_rad=quantile(trajectory,'yaw_error_rad'),pose_samples=len(trajectory),admitted_samples=len(qualified),admission_sim_s=active,admission_fraction=active/total if total else 0,states=list(dict.fromkeys(s['state'] for s in status)),wall_seconds=rt['wall_seconds'],physical_stops={})
 if rt['case']=='normal':checks['baseline_accuracy']=bool(len(qualified)>20 and m['xy_p95_m']<=.15 and m['yaw_p95_rad']<=.05 and m['all_published_xy_p95_m']<=.15 and m['all_published_yaw_p95_rad']<=.05)
 checks['diagnostic_motion']=read(out/'motion/result.json')['distance']>.5
 def any_state(state,start,end):return any(s['state']==state and start<s['t']<end for s in status)
 if rt['case']=='gnss_outage':
  checks.update(outage_map_takeover=any_state('MAP_ONLY',26,43) and all(s['max_speed']<=.06 for s in status if 26<s['t']<43),float_rejected=any(s['gnss_reason']=='float' for s in status if 51<s['t']<58) and all(s['max_speed']<=.06 for s in status if 51<s['t']<58),recovery=any_state('TRUSTED_MAP',60,rt['seconds']))
 if rt['case']=='false_fixed_and_loss':
  checks.update(false_fixed_innovation_rejected=any(s['gnss_reason']=='INNOVATION_REJECTED_FIXED' for s in status if 26<s['t']<38) and all(s['max_speed']<=.06 for s in status if 26<s['t']<38),dual_loss_stop=all(not s['navigation_allowed'] for s in status if 50<s['t']<64),recovery=any_state('TRUSTED_MAP',67,rt['seconds']))
 if rt['case']=='map_degradation':
  checks.update(map_loss_gnss_takeover=any_state('GNSS_ONLY',27,29),map_mismatch_held=any(s['plan_revalidation_required'] for s in status if 41<s['t']<54) and all(not s['navigation_allowed'] for s in status if 42<s['t']<54),recovery=any_state('TRUSTED_MAP',58,rt['seconds']))
 if rt['case']!='normal':
  after={'gnss_outage':60,'false_fixed_and_loss':67,'map_degradation':58}[rt['case']];checks['physical_motion_after_recovery']=any(c['mode']=='DRIVE' and c['v']>.03 for c in commands if c['t']>after)
 trigger=48 if rt['case']=='false_fixed_and_loss' else 40 if rt['case']=='map_degradation' else None
 if trigger:
  sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));spec=importlib.util.spec_from_file_location('physical_stop',BASE/'ws/src/chassis_bridge/scripts/analyze.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);physical=mod.physical(truth);closed=next((s['t'] for s in status if trigger<=s['t']<trigger+4 and not s['navigation_allowed']),None);score=mod.stop_score(physical,closed,closed+3) if closed else dict(passed=False);moving=any(c['mode']=='DRIVE' and c['v']>.03 for c in commands if trigger-3<c['t']<trigger);delay=closed-trigger+score['delay_sim_s'] if closed and 'delay_sim_s' in score else None;m['physical_stops']=dict(trigger_t=trigger,gate_t=closed,moving_before_fault=moving,fault_to_stop_s=delay,**score);checks['physical_stop']=bool(score['passed'] and moving and delay is not None and delay<2.)
 resources=read(out/'resources.json');m['peak_rss_bytes']=max(x['owned_rss_bytes'] for x in resources);checks['resource_limit']=m['peak_rss_bytes']<8589934592;m['passed']=bool(all(checks.values()));dump(out/'metrics.json',m)
 dump(out/'plot_data.json',dict(trajectory=trajectory,status=status,gnss=gps,maps=maps,commands=[{k:r[k] for k in ['t','mode','v','w','reason']} for r in commands],truth=[dict(t=r['t'],xyz=(np.array(r['xyz'])-[28,14,0]).tolist()) for r in truth[::10]]));return m
if __name__=='__main__':
 import sys
 m=analyze(Path(sys.argv[1]));print(json.dumps(m,indent=2));raise SystemExit(0 if m['passed'] else 1)
