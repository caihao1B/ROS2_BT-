"""Isolated two-point physical case. Genuine failure at its last selected point must ask an operator."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from spray_defer_candidate_r10 import replace_one
from spray_handoff_candidate_r10 import build as build_handoff

SCRIPT=Path(__file__).resolve();SCRIPTS=SCRIPT.parent
PAIR=['point_11_20','point_10_20']
REASON='MANUAL_OR_SAFETY_HOLD BOUNDED_REPLAN_EXHAUSTED'

def build_worker():
    parent=build_handoff();s=parent.read_text();s=replace_one(s,"'LOCAL_DOCK_NAV_REJECTED','LOCAL_DOCK_SAFETY_HOLD','LOCALIZATION_REVALIDATION_BOUND'}", "'LOCAL_DOCK_NAV_REJECTED','LOCAL_DOCK_SAFETY_HOLD','LOCALIZATION_REVALIDATION_BOUND','MANUAL_OR_SAFETY_HOLD BOUNDED_REPLAN_EXHAUSTED'}")
    old="    return dict(task_terminal='HUMAN_REQUIRED',mission_passed=False,process_exit=0 if all(checks.values()) else 2,"
    new="""    if request['reason']=='MANUAL_OR_SAFETY_HOLD BOUNDED_REPLAN_EXHAUSTED':
        proof=request.get('last_point_proof',{})
        checks['native_locked_last_original_point']=(proof.get('no_selected_alternative') is True and proof.get('observed_nav_aborts',0)>=3 and proof.get('actual_replan_decisions',0)>=2 and proof.get('native_action')=='HUMAN' and proof.get('reason_code')=='BOUNDED_REPLAN_EXHAUSTED' and proof.get('native_alternative') is False and proof.get('native_stopped_confirmed') is True and proof.get('native_session')==session and proof.get('native_version')==request['original_key'].split(':')[0] and proof.get('native_manual_latched') is True)
"""+old
    s=replace_one(s,old,new)
    old="            persist_handoff(handoff);log('HUMAN_ASSISTANCE_REQUESTED'"
    new="""            native=st['bt'] or {};local=events[point_events_start:] if 'point_events_start' in locals() else []
            handoff['last_point_proof']=dict(no_selected_alternative=not points[st['index']+1:],observed_nav_aborts=sum(e['event']=='NAV_TERMINAL_FAILURE' for e in local),
                actual_replan_decisions=sum(e['event']=='RETRY_DECISION' and e.get('action')=='REPLAN' for e in local),native_action=native.get('action'),reason_code=native.get('reason_code'),
                native_alternative=(native.get('evidence') or {}).get('alternative'),native_stopped_confirmed=native.get('stopped_confirmation'),native_manual_latched=native.get('manual_latched'),
                native_session=native.get('session'),native_version=native.get('task_version'),native_context_sequence=native.get('context_sequence'),native_dispatch_id=native.get('dispatch_id'))
"""+old
    s=replace_one(s,old,new);out=parent.parent.parent/'spray_last_blocked'/'mission_spray_last_blocked.py';out.parent.mkdir(parents=True,exist_ok=True);compile(s,str(out),'exec');out.write_text(s);return out

def isolated_runtime():
    from common import APP,PKG
    path=SCRIPTS/'spray_defer_fixture_r10.py';s=path.read_text()
    s=replace_one(s,'FOUR = ["point_11_20", "point_10_20", "point_9_20", "point_8_20"]','FOUR = ["point_11_20", "point_10_20"]')
    s=replace_one(s,'ids[i:i+4] != FOUR','ids[i:i+len(FOUR)] != FOUR');s=replace_one(s,'cluster["points"][i:i+4]','cluster["points"][i:i+len(FOUR)]')
    s=replace_one(s,'point_limit=4, case=profile','point_limit=len(FOUR), case=profile');s=replace_one(s,'profile = "bounded_defer" if obstacle else "bounded_defer_natural"','profile = "last_selected_equipment_blocked" if obstacle else "natural_last_selected"')
    ns={'__file__':str(path),'__name__':'isolated_two_point_fixture'};exec(compile(s,str(path),'exec'),ns)
    ns['EXPERIMENTS']=APP/'results/experiments/spray_last_blocked';ns['FIXTURE_SOURCES']=[*ns['FIXTURE_SOURCES'],SCRIPTS/'spray_handoff_candidate_r10.py',SCRIPT]
    cfg,pts=ns['original_points']()
    if [p['id'] for p in pts]!=PAIR:raise ValueError('Changed authorized original selection')
    return ns['build_runtime'](build_worker(),obstacle=True)

def analyze(case):
    from common import read,dump,sha
    from analyze_spray_r10 import analyze as quantify
    from analyze_r02 import rows
    mission=read(case/'mission_result.json') if (case/'mission_result.json').exists() else {};runtime=read(case/'runtime.json');spec=read(case/'spec.json');cfg=read(case/'mission.json');e=rows(case/'mission_events.jsonl');bt=rows(case/'decisions.jsonl');fx=read(case/'scene/fixture.json');q=quantify(case);jobs=q['jobs'];h=mission.get('operator_handoff') or {};s=h.get('shutdown') or {};proof=h.get('last_point_proof') or {};actual=next((p['exit'] for p in runtime['processes'] if p['role']=='observer'),None);d=next((d for d in bt if d['action']=='HUMAN' and d['reason_code']=='BOUNDED_REPLAN_EXHAUSTED' and d['dispatch_id']==proof.get('native_dispatch_id') and d.get('stopped_confirmation') and d['evidence']['alternative'] is False),None)
    f=rows(case/'video/frames.jsonl');camera=read(case/'video/recording.json');gates=rows(case/'control/gate.jsonl');start=next((g['t'] for g in gates if g['mode']=='DRIVE' and abs(g['v'])+abs(g['w'])>.003),None)
    if start is None:start=next((v['t'] for v in e if v['event']=='NAV_GOAL'),None)
    active=[v for v in f if (start or 0)<=v['sim_t']<=mission.get('t',0)];gap=max((b['sim_t']-a['sim_t'] for a,b in zip(active,active[1:])),default=1e9);age=max((v['image_wall_age_s'] for v in active),default=1e9)
    gates={k:q['checks'].get(k) for k in ('runtime_clean','map_database_unchanged','sensor_render_configuration_valid','four_real_near_sensors','map_pose_rmse_under_15cm','measured_final_stop','unique_actuator_owner','unique_tool_owner','unique_transform_owners','implementation_bound','no_unexpected_contact')}
    gates.update(original_and_selected_accurate=spec['candidate_selected_point_ids']==PAIR and spec['point_limit']==2 and mission.get('selected_points')==2 and mission.get('original_points')==56 and len(mission.get('not_selected',[]))==54,
        last_point_really_occupied=bool(fx.get('physical_fixture') and fx['physical_fixture']['map_xy']==[3.5,47.5] and fx['obstacle_in_frozen_prior'] is False),human_dispatch_and_no_alternative=bool(d) and proof.get('observed_nav_aborts',0)>=3 and proof.get('actual_replan_decisions',0)>=2 and proof.get('no_selected_alternative') is True,
        process_clean_task_unsuccessful=actual==0 and mission.get('passed') is False and mission.get('terminal')=='HUMAN_REQUIRED' and s.get('shutdown_verified') is True and s.get('process_exit')==0 and all(s.get('checks',{}).values()),
        zero_work_credit_on_blocked_original=h.get('point')==PAIR[-1] and h.get('original_key')==cfg['version']+':'+PAIR[-1] and h.get('residual')==3. and h.get('credit_added_by_this_handoff')==0. and h.get('previous_confirmed')==0. and not h.get('automatic_resume',True) and not any(j['key'].endswith(':'+PAIR[-1]) for j in jobs),
        first_point_actual_feedback=len(jobs)==1 and jobs[0]['key']==cfg['version']+':'+PAIR[0] and jobs[0]['state']=='SUCCEEDED' and jobs[0]['requested']<=jobs[0]['confirmed']<=jobs[0]['physical_dose']+.1 and jobs[0]['maximum_physical_docking_error_m']<.20,
        camera_covers_work_and_hold=bool(camera.get('valid') and camera.get('encoder_exit')==0 and camera.get('source')=='actual Gazebo camera' and len(f)==camera['frames'] and start is not None and f[0]['sim_t']<=start and f[-1]['sim_t']>=mission.get('t',1e20)-.5 and gap<=1.0 and age<=2.5))
    result=dict(schema='SPRAY-LAST-SELECTED-HUMAN-CASE/1',session=case.name,passed=all(gates.values()),checks={k:bool(v) for k,v in gates.items()},formal_credit=False,trigger_phase='COARSE_NAVIGATION' if d else None,precision_handoff_validated=False,
        selected=PAIR,original_count=mission.get('original_points'),unselected_count=len(mission.get('not_selected',[])),completed=[v['point'] for v in mission.get('completed',[])],
        error=mission.get('error'),mission_passed=mission.get('passed'),terminal=mission.get('terminal'),observed_process_exit=actual,sim_s=mission.get('t'),runtime_clean=runtime.get('passed'),actual_jobs=jobs,operator_handoff=h,source_sha256={n:sha(case/n) for n in ('spec.json','mission.json','mission_result.json','operator_handoff.json','mission_events.jsonl','decisions.jsonl','runtime.json','video/full.mp4','video/recording.json') if (case/n).is_file()},video=dict(max_active_gap_s=gap,max_active_age_s=age))
    dump(case/'candidate_handoff_acceptance.json',result);return result

if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('mode',choices=('check','run'));ap.add_argument('--seconds',type=int,default=155);ap.add_argument('--rtf',type=float,default=.55);a=ap.parse_args();runtime=isolated_runtime()
    if a.mode=='check':print('two original keys, separate generated runtime and worker bound');raise SystemExit(0)
    case=runtime(seconds=a.seconds,rtf=a.rtf);r=analyze(case);print(json.dumps({k:r[k] for k in ('session','passed','terminal','observed_process_exit','sim_s','completed')}));raise SystemExit(0 if r['passed'] else 2)
