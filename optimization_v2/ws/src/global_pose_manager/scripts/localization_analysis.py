"""Offline fixed-frame scoring. This module alone reads independent truth."""
import importlib.util,sys
from s07_common import *
from mapping_analysis import rows
def analyze(out):
    out=Path(out);rt=read(out/'runtime.json');obs=rows(out/'online/observations.jsonl');end=read(out/'online/result.json');truth=rows(out/'evaluation/truth.jsonl');commands=rows(out/'motion/commands.jsonl');frames=rows(out/'mapping_inputs/frames.jsonl');cfg=yaml.safe_load((BASE/'config/map_contract.yaml').read_text());limits=cfg['scoring'];checks={}
    status=[r for r in obs if r['key']=='status'];poses=[r for r in obs if r['key']=='pose'];evidence=[r for r in obs if r['key']=='evidence'];good=[r for r in evidence if r['valid']];tt=np.array([r['t'] for r in truth]);xyz=np.array([r['xyz'] for r in truth]);yaw=np.unwrap(np.array([r['rpy'][2] for r in truth]));st=np.array([r['t'] for r in status]);allowed=[r for r in status if r['navigation_allowed']]
    trajectory=[]
    for r in poses:
        t=r['t'];p=np.array(r['pose']);ref=np.array([np.interp(t,tt,xyz[:,i]) for i in range(3)])-[28.,14.,0.];heading=float(Rotation.from_quat(p[3:]).as_euler('xyz')[2]);s=status[max(0,int(np.searchsorted(st,t,side='right'))-1)]
        trajectory.append(dict(t=t,estimate=p[:3].tolist(),truth=ref.tolist(),xy_error_m=float(np.linalg.norm(p[:2]-ref[:2])),yaw_error_deg=abs(math.degrees(wrap(heading-np.interp(t,tt,yaw)))),state=s['state'],allowed=s['navigation_allowed']))
    qualified=[r for r in trajectory if r['allowed']];xy=np.array([r['xy_error_m'] for r in qualified]);heading=np.array([r['yaw_error_deg'] for r in qualified]);first=min((s['t'] for s in allowed),default=None)
    metrics=dict(case=rt['case'],pose_name=rt['pose_name'],checks=checks,first_localized_sim_s=first,states=list(dict.fromkeys(s['state'] for s in status)),accepted_quality=len(good),quality_total=len(evidence),qualified_pose_samples=len(qualified),xy_rmse_m=float(np.sqrt(np.mean(xy**2))) if len(xy) else None,xy_p95_m=float(np.quantile(xy,.95)) if len(xy) else None,xy_max_m=float(xy.max()) if len(xy) else None,yaw_rmse_deg=float(np.sqrt(np.mean(heading**2))) if len(heading) else None,yaw_max_deg=float(heading.max()) if len(heading) else None,allowed_fraction=len(allowed)/max(1,len(status)),physical_stops={},events=rt.get('events',[]),database_unchanged=rt['database_unchanged'])
    checks['capture_versions_match']=read(out/'capture_inputs.json')==capture_inputs()
    intervals=[(max(0.,b['t']-max(a['t'],first)),a['navigation_allowed']) for a,b in zip(status,status[1:]) if first is not None and b['t']>=first];duration=sum(dt for dt,allow in intervals);admitted=sum(dt for dt,allow in intervals if allow)
    metrics['admission_fraction_after_first_fix']=admitted/duration if duration else 0.;metrics['admission_duration_sim_s']=admitted;metrics['post_fix_duration_sim_s']=duration
    checks['completed']=rt['success'] and end['completed'];checks['localized_with_multiple_matches']=len(good)>=3 and first is not None and len(qualified)>10
    checks['position_and_heading_accuracy']=len(qualified)>10 and metrics['xy_rmse_m']<limits['max_xy_rmse_m'] and metrics['yaw_rmse_deg']<limits['max_yaw_rmse_deg']
    checks['acquisition_bounded']=first is not None and first<limits['max_acquisition_sim_s']
    checks['frozen_database_unchanged']=rt['database_unchanged'];checks['bundle_valid']=bool(verify_bundle())
    checks['confirmation_hysteresis']=all(s['confirmation_count']>=cfg['manager']['confirmation_count'] for s in allowed)
    checks['no_unadmitted_drive']=all(c['status'] and c['status']['navigation_allowed'] for c in commands if c['mode']=='DRIVE')
    checks['unique_tf_and_actuator']=bool(end['audits']) and all(sorted(r['tf'])==['s05_local_odometry','s07_global_pose_manager'] and r['actuator']==['s03_drive_bridge'] for r in end['audits'] if r['t']>10) and {(r['parent'],r['child']) for r in end.get('tf_edges',[])}=={('map','odom'),('odom','base_link')}
    checks['truth_isolation']=not any('/evaluation/' in topic or '/s04/raw/' in topic or topic.startswith('/world/') for topic in end['subscriptions'])
    checks['coarse_prior_not_truth_seed']=False
    initial=next((r for r in rt['events'] if r['event']=='initial_hint'),None)
    if initial:
        xy0=xyz[np.argmin(abs(tt-5)),:2]-[28.,14.];yaw0=float(np.interp(5,tt,yaw));prior=initial['pose'];prior_yaw=Rotation.from_quat(prior[3:]).as_euler('xyz')[2];metrics['initial_prior_error_m']=float(np.linalg.norm(np.array(prior[:2])-xy0));metrics['initial_prior_yaw_error_deg']=abs(math.degrees(wrap(prior_yaw-yaw0)));checks['coarse_prior_not_truth_seed']=metrics['initial_prior_error_m']>.3 and metrics['initial_prior_yaw_error_deg']>3
    validframes=[f for f in frames if 'raw' in f];checks['aligned_observation_epochs']=bool(validframes) and all(f['pair_error_s']==0 and f['max_sync_error_s']<=.04 and f.get('alignment','').startswith('Rigid') for f in validframes)
    checks['sensor_based_map_agreement']=bool(good) and all(r['surface_overlap']>=cfg['quality']['min_overlap_fraction'] and r['surface_p95_m']<=cfg['quality']['max_surface_p95_m'] for r in good)
    contacts=read(out/'evaluation/contacts.json');metrics['obstacle_contacts']={k:v for k,v in contacts.items() if 'vehicle::' in k and not any(t in k for t in ['ground::','grade_'])};checks['no_obstacle_contact']=not metrics['obstacle_contacts']
    checks['diagnostic_motion_observed']=read(out/'motion/result.json')['distance']>.5
    resources=read(out/'resources.json');metrics['peak_owned_rss_bytes']=max(x['owned_rss_bytes'] for x in resources);metrics['wall_seconds']=rt['wall_seconds'];metrics['rtf']=rt['seconds']/rt['wall_seconds'];checks['resource_bound']=metrics['peak_owned_rss_bytes']<8589934592
    if rt['case']=='restart_occlusion':
        events={r['event']:r for r in rt['events']};metrics['restart_recovery_sim_s']=next((s['t']-events['slam_new_process']['t'] for s in status if 'slam_new_process' in events and s['t']>=events['slam_new_process']['t'] and s['navigation_allowed']),None);metrics['occlusion_recovery_sim_s']=next((s['t']-85 for s in status if s['t']>=85 and s['navigation_allowed']),None)
        checks['actual_process_restart']='slam_stopped' in events and 'slam_new_process' in events and events['slam_stopped']['exit_code']==0
        checks['localized_after_restart']=any(51<s['t']<70 and s['navigation_allowed'] for s in status);checks['localized_after_occlusion']=any(s['t']>87 and s['navigation_allowed'] for s in status)
        checks['occlusion_delivered']=any(f.get('occluded',False) for f in validframes);checks['lost_state_observed']=any(70<s['t']<88 and s['state']=='LOST' for s in status)
        sys.path.insert(0,str(BASE/'ws/src/chassis_bridge/scripts'));spec=importlib.util.spec_from_file_location('s07_stop_score',BASE/'ws/src/chassis_bridge/scripts/analyze.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);physical=module.physical(truth)
        for name,t in [('restart',events.get('slam_stopped',{}).get('t',40.)),('occlusion',70.)]:
            closed=next((s['t'] for s in status if t<=s['t']<t+8 and not s['navigation_allowed']),None)
            moving=[c for c in commands if t-3<c['t']<t and c['mode']=='DRIVE' and c['v']>.03];score=module.stop_score(physical,closed,closed+3) if closed is not None else dict(passed=False)
            metrics['physical_stops'][name]=dict(trigger_t=t,gate_closed_t=closed,moving_before_fault=bool(moving),fault_to_stop_sim_s=(closed-t+score['delay_sim_s']) if closed is not None and 'delay_sim_s' in score else None,**score)
        checks['physical_fault_stops']=all(s['passed'] and s['moving_before_fault'] and s['fault_to_stop_sim_s'] is not None and s['fault_to_stop_sim_s']<limits['stop_delay_s'] for s in metrics['physical_stops'].values())
    if rt['case']=='wrong_initial':
        checks['wrong_seed_not_blindly_admitted']=bool(qualified) and all(r['xy_error_m']<limits['max_xy_rmse_m'] for r in qualified if r['t']<30)
        metrics['recovered_before_operator_correction']=first is not None and first<30
        pre_infos=[r for r in obs if r['key']=='info' and 10<r['t']<30];no_verified_candidate=len(pre_infos)>=3 and not any(r['allowed'] and r['t']<30 for r in trajectory) and not any(r['valid'] and r['t']<30 for r in evidence);metrics['wrong_seed_diagnosis']='NO_VERIFIED_SENSOR_MATCH' if no_verified_candidate else 'REJECTED_CANDIDATE_OR_HYPOTHESIS'
        checks['wrong_seed_diagnosed']=no_verified_candidate or any(not r['valid'] for r in evidence) or any(r['key']=='info' and r['stats'].get('Loop/RejectedHypothesis/',0)>0 for r in obs)
    metrics['passed']=bool(all(checks.values()));dump(out/'metrics.json',metrics);dump(out/'plot_data.json',dict(trajectory=trajectory,status=status,evidence=evidence,commands=[{k:r[k] for k in ['t','mode','v','w','reason']} for r in commands],truth=[dict(t=r['t'],xyz=(np.array(r['xyz'])-[28.,14.,0.]).tolist()) for r in truth[::10]]));return metrics
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();m=analyze(a.out);print(json.dumps(m,ensure_ascii=False));raise SystemExit(0 if m['passed'] else 1)
