"""Compare A1 methods from closed sessions, counting original physical work and elapsed sim time."""
import argparse
import hashlib
import json
from pathlib import Path
from xml.etree import ElementTree as ET

from analyze_spray_r10 import analyze
from common import ROOT, dump, read, sha


def rows(path):
    with path.open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def scene_digest(run):
    tree=ET.parse(run/'scene/world.sdf')
    for node in tree.findall('.//plugin/session'):
        if node.text!=run.name:
            raise ValueError('Unexpected scene session')
        node.text='RUN_SESSION'
    tool=tree.find(".//plugin[@name='s10::ToolActuator']/config")
    cfg=json.loads(tool.text)
    for key in ('session','tool_revision','plan_revision'):
        cfg.pop(key)
    tool.text=json.dumps(cfg,sort_keys=True)
    data=ET.canonicalize(ET.tostring(tree.getroot(),encoding='unicode'),strip_text=True)
    return hashlib.sha256(data.encode()).hexdigest()


def incomplete(run,variant,cfg,spec,receipt,missing):
    chosen=spec.get('candidate_selected_point_ids',[])
    cluster=next((c for c in cfg.get('clusters',[]) if c['id']=='A1'),{})
    goals={p['id']:p for p in cluster.get('points',[])}
    done={r['point']:r for r in receipt.get('completed',[])}
    return dict(variant=variant,session=run.name,path=str(run.relative_to(ROOT)),scope='original A1 selection',
                disposition=receipt.get('terminal') or 'INCOMPLETE_STARTUP_OR_EVIDENCE',
                error=receipt.get('error'),missing_or_invalid_evidence=missing,first_nav_t=None,
                elapsed_sim_s=receipt.get('t'),first_goal_to_last_event_sim_s=None,wall_seconds=receipt.get('wall_seconds'),
                completed_count=len(done),selected_count=len(chosen),total_original=len(goals) or spec.get('original_obligations'),
                points=[dict(id=k,requested=goals.get(k,{}).get('quantity'),completed=k in done,
                             confirmed=done.get(k,{}).get('confirmed',0),physical_dose=None,
                             maximum_physical_docking_error_m=None,cycle_sim_s=None) for k in chosen],
                measured_subset_pass=False,formal_credit=False,version_and_executable_bound=False,
                quantitative_safety={},camera={'valid':False},terrain={},paired_eligible=False,
                scene_comparison_sha256=None,original_contract_sha256=None)


def evaluate(run,variant):
    run=Path(run).resolve()
    def optional(name):
        return read(run/name) if (run/name).is_file() else {}
    spec=optional('spec.json')
    receipt=optional('mission_result.json')
    cfg=optional('mission.json')
    runtime=optional('runtime.json')
    required=('mission_result.json','runtime.json','spec.json','mission.json','mission_events.jsonl','risk/risk.jsonl',
              'control/gate.jsonl','decisions.jsonl','evaluation/truth.jsonl','evaluation/contacts.json','video/full.mp4',
              'video/frames.jsonl','video/recording.json','scene/world.sdf','near/near.jsonl','server.log')
    missing=[n for n in required if not (run/n).is_file()]
    if missing:
        return incomplete(run,variant,cfg,spec,receipt,missing)
    if not any(e.get('event')=='NAV_GOAL' for e in rows(run/'mission_events.jsonl')):
        return incomplete(run,variant,cfg,spec,receipt,['No NAV_GOAL was sent'])
    audit=analyze(run)
    checks=audit['checks']
    essential=('runtime_clean','map_database_unchanged','no_unexpected_contact','sensor_render_configuration_valid',
               'four_real_near_sensors','map_pose_rmse_under_15cm','measured_final_stop','unique_actuator_owner',
               'unique_tool_owner','unique_transform_owners','implementation_bound')
    events=list(rows(run/'mission_events.jsonl'))
    targets={p['id']:p for c in cfg['clusters'] if c['id']=='A1' for p in c['points']}
    chosen=spec['candidate_selected_point_ids']
    jobs={j['key'].split(':')[-1]:j for j in audit['jobs']}
    start=next(e['t'] for e in events if e['event']=='NAV_GOAL')
    end=receipt['t']
    decisions=[e for e in rows(run/'decisions.jsonl') if start<=e['t']<=end]
    health={}
    for row in rows(run/'risk/risk.jsonl'):
        h=row.get('terrain_health') or {}
        if start<=h.get('t',-1)<=end:
            health[h['t']]=h
    waits=[h.get('pair_wait_sim_s',0) for h in health.values() if h.get('valid')]
    deltas=[h['odom_delta_s'] for h in health.values() if h.get('valid')]
    has_defer_flag=any('paired_after_defer' in h for h in health.values())
    exact_waits=sum(h.get('paired_after_defer') is True for h in health.values() if h.get('valid')) if has_defer_flag else None
    terrain=dict(exact_observed_deferred_matches=exact_waits,recorded_samples=len(health),valid_samples=sum(bool(h.get('valid')) for h in health.values()),
                 odometry_epoch_mismatches=sum(h.get('reason')=='ODOMETRY_EPOCH_MISMATCH' for h in health.values()),
                 waited_then_matched=sum(x>0 for x in waits),max_pair_wait_sim_s=max(waits,default=0),
                 maximum_accepted_pair_delta_s=max(deltas,default=None),
                 perception_cancel_dispatches=len({d['dispatch_id'] for d in decisions if d['action']=='CANCEL' and d['reason_code']=='PERCEPTION_ADMISSION_WAIT'}))
    cam=read(run/'video/recording.json')
    frames=list(rows(run/'video/frames.jsonl'))
    first_drive=next((g['t'] for g in rows(run/'control/gate.jsonl') if g['mode']=='DRIVE' and abs(g['v'])+abs(g['w'])>.003),None)
    active=[f for f in frames if first_drive is not None and first_drive<=f['sim_t']<=end]
    gap=max((n['sim_t']-p['sim_t'] for p,n in zip(active,active[1:])),default=None)
    age=max((f['image_wall_age_s'] for f in active),default=None)
    camera=dict(frames=len(frames),internal_valid=cam['valid'],max_active_gap_sim_s=gap,max_active_age_wall_s=age)
    camera['valid']=bool(cam['valid'] and cam['source']=='actual Gazebo camera' and cam['encoder_exit']==0 and frames and
                         len(frames)==cam['frames'] and first_drive is not None and frames[0]['sim_t']<=first_drive and frames[-1]['sim_t']>=end-.5 and
                         gap is not None and gap<=1. and age is not None and age<=2.5)
    pts=[]
    for point in chosen:
        selected=[e for e in events if e.get('point')==point]
        first=next((e['t'] for e in selected if e['event']=='NAV_GOAL'),None)
        success=next((e['t'] for e in selected if e['event']=='TOOL_RESULT' and e['state']=='SUCCEEDED' and not e['replayed']),None)
        initial_admit=next((e['t'] for e in selected if e['event']=='DOCKING_SETTLED_CHECK' and e['stable']),None)
        j=jobs.get(point,{})
        stable=[e for e in selected if e['event']=='DOCKING_SETTLED_CHECK' and e['stable']]
        pts.append(dict(id=point,target=targets[point]['target'],requested=targets[point]['quantity'],confirmed=j.get('confirmed',0),
                        physical_dose=j.get('physical_dose',0),maximum_physical_docking_error_m=j.get('maximum_physical_docking_error_m'),
                        ledger_state=j.get('state'),uncertain=j.get('uncertain'),first_nav_t=first,succeeded_t=success,
                        cycle_sim_s=success-first if success is not None and first is not None else None,
                        time_to_service_sim_s=initial_admit-first if initial_admit is not None and first is not None else None,
                        completed=success is not None,route_goals=sum(e['event']=='NAV_GOAL' for e in selected),
                        local_corrections=sum(e['event']=='LOCAL_DOCK_GOAL' for e in selected),
                        global_reacquisitions=sum(e['event']=='GLOBAL_PRECISION_REACQUIRE_GOAL' for e in selected),
                        settled_windows=[{k:e['stability'][k] for k in e['stability']} for e in stable]))
    sources=('spec.json','mission.json','mission_result.json','runtime.json','mission_events.jsonl','ledger/ledger.sqlite',
             'risk/risk.jsonl','tool_evaluation/physical.jsonl','evaluation/truth.jsonl','evaluation/contacts.json','video/full.mp4','video/recording.json')
    original=receipt['original_points']
    bound=spec['implementation_bindings']
    worker=next((p for p,v in bound.items() if v==spec['online_probe_sha256'] and p.endswith('.py')),None)
    matching=bool(worker and (ROOT/worker).is_file() and sha(ROOT/worker)==spec['online_probe_sha256'] and
                  spec['candidate_mission_version']==cfg['version'] and cfg['version']==read(run/'scene/tool.json')['plan_revision'])
    passed=bool(receipt.get('passed') and len(pts)==len(jobs)==len(receipt['completed']) and set(jobs)==set(chosen) and
                original==len(targets)==56 and camera['valid'] and matching and all(checks[k] for k in essential) and
                checks['feedback_and_independent_physics_agree'] and checks['idempotent_replay_no_extra_flow'])
    original_contract={k:v for k,v in cfg.items() if k not in ('version','bounded_defer_experiment','precision_session')}
    original_hash=hashlib.sha256(json.dumps(original_contract,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return dict(variant=variant,original_contract_sha256=original_hash,session=run.name,path=str(run.relative_to(ROOT)),scope='original A1 consecutive points only',source_version=cfg['version'],
                elapsed_sim_s=end,first_goal_to_last_event_sim_s=end-start,first_nav_t=start,wall_seconds=runtime['wall_seconds'],
                completed_count=len(receipt['completed']),selected_count=len(chosen),total_original=original,
                disposition=receipt.get('terminal') or ('SUCCEEDED' if receipt.get('passed') else 'INCOMPLETE'),error=receipt.get('error'),
                measured_subset_pass=passed,formal_credit=False,scene_comparison_sha256=scene_digest(run),points=pts,terrain=terrain,
                quantitative_safety={k:checks[k] for k in essential},camera=camera,independent_dose_agreement=checks['feedback_and_independent_physics_agree'],
                replay_no_flow=checks['idempotent_replay_no_extra_flow'],version_and_executable_bound=matching,map_xy_rmse_m=audit['map_xy_rmse_m'],
                low_speed_capture_proposals=sum((e.get('evidence') or {}).get('reason')=='FRESH_LOW_SPEED_INSIDE_NAV_RADIUS' for e in events),
                hashes={str(f):sha(run/f) for f in sources if (run/f).is_file()})


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--candidate',type=Path,action='append',required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    records=[evaluate(a.baseline.resolve(),'baseline')]
    records.extend(evaluate(c.resolve(),'candidate') for c in a.candidate)
    paired=[]
    for candidate in records[1:]:
        base=records[0]
        if base['original_contract_sha256'] is None or candidate['original_contract_sha256'] is None or base['original_contract_sha256']!=candidate['original_contract_sha256'] or base['scene_comparison_sha256']!=candidate['scene_comparison_sha256'] or [q['id'] for q in base['points']]!=[q['id'] for q in candidate['points']]:
            continue
        measurements=[]
        for old,new in zip(base['points'],candidate['points']):
            if old['completed'] and new['completed']:
                measurements.append(dict(id=old['id'],baseline_sim_s=old['cycle_sim_s'],candidate_sim_s=new['cycle_sim_s'],
                                         saved_sim_s=old['cycle_sim_s']-new['cycle_sim_s']))
        paired.append(dict(baseline=base['session'],candidate=candidate['session'],paired_completed=measurements,
                           common_points_sum_saved_sim_s=sum(v['saved_sim_s'] for v in measurements)))
    dump(a.out,dict(schema='R10-A1-PRECISION-COMPARISON/1',runs=records,paired=paired,formal_credit=False,
                    time_basis='Each per-point cycle spans its first NavigateToPose to its confirmed physical ToolTask success. Incomplete cycles are null. Control and rendering schedules vary across runs.',
                    comparison_scene_exclusions=['runtime session keys','tool plan/revision identity fields']))
    print(str(a.out))


if __name__=='__main__':
    main()
