"""Assess the experimental selection, raw native decisions, physical effects and camera independently."""
from pathlib import Path
from analyze_spray_r10 import analyze as physical_analysis
from common import read,dump,sha,ROOT
from analyze_r02 import rows

def analyze(out):
    out=Path(out)
    if out.parent.name!='spray_defer':raise ValueError('Analysis only accepts separated experimental results')
    spec=read(out/'spec.json');cfg=read(out/'mission.json');runtime=read(out/'runtime.json');mission=read(out/'mission_result.json') if (out/'mission_result.json').is_file() else {}
    baseline=physical_analysis(out);sub=spec['candidate_selected_point_ids'];events=rows(out/'mission_events.jsonl');decisions=rows(out/'decisions.jsonl');jobs=baseline['jobs']; completed={x['point']:x for x in mission.get('completed',[])}
    deferred=[e for e in events if e['event']=='CANDIDATE_DEFER_CONFIRMED']; proofs=[e for e in events if e['event']=='CANDIDATE_EXHAUSTION_EVIDENCE']; physical=rows(out/'tool_evaluation/physical.jsonl');matched=[]
    for e in deferred:
        key=cfg['version']+':'+e['point']; other=e['next_point'];later=next((a for a in events if a['event']=='TOOL_RESULT' and a['t']>e['t'] and a.get('point')==other and a.get('state')=='SUCCEEDED' and not a.get('replayed')),None)
        native=next((d for d in decisions if d.get('session')==spec['session'] and d.get('task_version')==cfg['version'] and d.get('dispatch_id')==e['dispatch_id'] and d.get('context_sequence')==e['context_sequence'] and d['action']=='DEFER' and d['reason_code']==e['reason'] and d['stopped_confirmation'] and d['t']<=e['t']+.03),None)
        predecessor=next((r for r in proofs if r['point']==e['point'] and r['stage']==e['stage'] and r['t']<=e['t']),None)
        stopped=[s for s in physical if e['t']-.18<=s['t']<=e['t'] and s['flow']==0. and abs(s['rpm_rad_s'])<.1];later_job=next((j for j in jobs if j['key']==cfg['version']+':'+other),None)
        residual=next((r for r in mission.get('failed',[]) if r['point']==e['point']),{})
        matched.append(dict(point=e['point'],next_point=other,stage=e['stage'],sim_s=e['t'],native_dispatch_id=e['dispatch_id'],native_dispatch_matched=bool(native),proof=predecessor,
          gate_checks=e['checks'],physical_stop_samples=len(stopped),remaining=residual.get('remaining'),deferred_dose_zero=(residual.get('confirmed')==0. and not any(j['key']==key for j in jobs) and not any(x.get('key')==key and x['flow']>0.01 for x in physical)),
          subsequent_meter_event=later,subsequent_physical_feedback=later_job))
    checks={k:v for k,v in baseline['checks'].items() if k in ('runtime_clean','map_database_unchanged','no_unexpected_contact','sensor_render_configuration_valid','four_real_near_sensors','map_pose_rmse_under_15cm','measured_final_stop','unique_actuator_owner','unique_tool_owner','unique_transform_owners','implementation_bound','idempotent_replay_no_extra_flow')}
    allpoints=[p['id'] for c in cfg['clusters'] if c['id']=='A1' for p in c['points']]; selected=set(sub); checks.update(full_denominator_retained=len(allpoints)==mission.get('original_points')==56 and mission.get('selected_points')==4 and set(mission.get('not_selected',[]))==set(allpoints)-selected,
      candidate_separate_version=cfg['version']!=cfg['bounded_defer_experiment']['parent_version'] and not spec['formal_credit'],
      one_explicit_residual=len(matched)==1 and bool(matched[0]['remaining']) and matched[0]['deferred_dose_zero'] if matched else False,
      native_decision_and_stops=bool(matched) and all(x['native_dispatch_matched'] and x['proof'] and x['proof']['proof']['eligible'] and all(x['gate_checks'].values()) and x['physical_stop_samples']>=2 for x in matched),
      next_original_point_paid=bool(matched) and all(x['subsequent_meter_event'] and x['next_point'] in completed and x['subsequent_physical_feedback'] and x['subsequent_physical_feedback']['physical_dose']+.1>=x['subsequent_physical_feedback']['confirmed'] and x['subsequent_physical_feedback']['maximum_physical_docking_error_m']<.20 for x in matched),
      all_selected_accounted=len(completed)+len(matched)==len(sub) and not mission.get('error'),all_paid_physically_verified=bool(jobs) and len(jobs)==len(completed) and all(j['state']=='SUCCEEDED' and not j['uncertain'] and j['requested']<=j['confirmed']<=j['physical_dose']+.1 and j['maximum_physical_docking_error_m'] is not None and j['maximum_physical_docking_error_m']<.20 for j in jobs))
    camera=read(out/'video/recording.json') if (out/'video/recording.json').exists() else {};frames=rows(out/'video/frames.jsonl');terminal=mission.get('t',1e20);gates=rows(out/'control/gate.jsonl');began=next((g['t'] for g in gates if g['mode']=='DRIVE' and abs(g['v'])+abs(g['w'])>.003),None);active=[f for f in frames if (began or 0)<=f['sim_t']<=terminal];gap=max((b['sim_t']-a['sim_t'] for a,b in zip(active,active[1:])),default=1e9);age=max((f['image_wall_age_s'] for f in active),default=1e9)
    checks['camera_covers_real_run']=bool(camera.get('valid') and camera.get('source')=='actual Gazebo camera' and camera.get('encoder_exit')==0 and len(frames)==camera['frames'] and began is not None and frames[0]['sim_t']<=began and frames[-1]['sim_t']>=terminal-.5 and gap<=1. and age<=2.5)
    passed=all(checks.values()); result=dict(schema='R10-EXPERIMENTAL-POINT-DEFER/1',session=spec['session'],case=spec['case'],behavior_result='PASS' if passed else 'PARTIAL',formal_credit=False,formal_mission_unchanged=True,formal_A1_latest_at_design='22/56 PARTIAL',
      precision_defer_demonstrated=passed and any(x['stage']=='PRECISION_DOCK' for x in matched),coarse_defer_demonstrated=passed and any(x['stage']=='COARSE_NAVIGATION' for x in matched),
      selected=sub,original_count=len(allpoints),completed=list(completed),residuals=mission.get('failed',[]),mission_error=mission.get('error'),runtime=runtime.get('passed'),checks={k:bool(v) for k,v in checks.items()},deferred=matched,
      evidence={f:sha(out/f) for f in ['spec.json','mission.json','scene/fixture.json','mission_events.jsonl','decisions.jsonl','mission_result.json','runtime.json','video/full.mp4','video/recording.json'] if (out/f).is_file()},video=dict(max_active_gap_sim_s=gap,max_active_age_wall_s=age,frames=len(frames)),physical_jobs=jobs)
    dump(out/'candidate_acceptance.json',result);return result
