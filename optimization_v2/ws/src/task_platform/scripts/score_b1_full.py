"""Read a B1 full-entrance session and score physical stops and cutting against its frozen mission."""
import argparse
from bisect import bisect_left, bisect_right
from collections import defaultdict
import gzip
import json
import math
from pathlib import Path

from shapely import union_all
from shapely.geometry import LineString, Point, shape

from analyze_mowing_r10 import analyze
from common import ROOT, canonical, dump, read, sha
from safe_plan_r10 import requested_target
from score_b1_connector import pose_error, rows, sampled, speed_and_distance

FROZEN = ROOT / 'validation/r10_continuation_20261119/b1'


def check_scope(session):
    spec = read(session/'spec.json')
    cfg = read(session/'mission.json')
    frozen = json.loads(gzip.decompress((FROZEN/'mission_46.json.gz').read_bytes()))
    frozen_receipt = read(FROZEN/'full_plan_preflight.json')
    cluster = next(c for c in cfg['clusters'] if c['cluster']=='B1')
    segments = cluster['segments']
    fixture = read(session/'scene/fixture.json')
    scope = (spec['task']=='mowing' and spec['cluster']=='B1' and spec['case']=='nominal'
             and spec['segment_start']==0 and spec['segment_limit']==0 and spec['sim_seconds']==cluster['budget_s']
             and spec['session']==session.name and fixture.get('selection')=='ALL_46'
             and fixture['spawn']==segments[0]['poses_world'][0] and fixture.get('online_access') is False
             and cfg == frozen and cfg['version']==canonical({k:v for k,v in cfg.items() if k!='version'})
             and sha(FROZEN/'mission_46.json.gz')==frozen_receipt['compressed_full_edition']['sha256']
             and [s['id'] for s in segments]==frozen_receipt['full_ordered_sections'])
    if not scope:
        raise ValueError('FULL_46_ENTRANCE_SCOPE_MISMATCH: separate editions and isolated connector trials cannot be credited')
    return spec, cfg, cluster


class Timeline:
    def __init__(self, records):
        self.records = records
        self.t = [r['t'] for r in records]
        if any(b < a for a,b in zip(self.t,self.t[1:])):
            raise ValueError('Physical observation time moved backwards')

    def window(self, end, span):
        a = bisect_left(self.t,end-span-1e-6)
        b = bisect_right(self.t,end+1e-6)
        return self.records[a:b]


def stop_row(segment, events, mission, cfg, timelines, previous_direction, next_direction):
    sid = segment['id']
    nav = cfg['navigation_execution']
    limits = nav['stopped_arrival_confirmation']
    safety = cfg['vehicle']['safety']
    row = dict(segment=sid,kind=segment['kind'],gear=segment['direction'],
               gear_changed_at_entry=previous_direction is not None and segment['direction']!=previous_direction,
               gear_changes_after=next_direction is not None and segment['direction']!=next_direction,
               endpoint_map=segment['poses'][-1],state='NOT_STARTED')
    if not events:
        return row
    row['state']='ATTEMPTED'
    completed = sid in mission['completed_segments']
    selected = [e for e in events if e['event']=='STOPPED_ORIGINAL_ARRIVAL_CONFIRMED']
    row['worker_completed']=completed
    if not selected:
        return row
    held = selected[-1]
    end,span=held['t'],held['span_s']
    truth,rotor,gate=(tl.window(end,span) for tl in timelines)
    goal=segment['poses_world'][-1]
    errors=[pose_error(p['xyz'],p['rpy'][2],goal) for p in truth]
    _,_,max_w=speed_and_distance(truth)
    max_v=max((abs(r['body_v']) for r in rotor),default=None)
    err_xy=max((p[0] for p in errors),default=None)
    err_yaw=max((p[1] for p in errors),default=None)
    max_rotor=max((abs(p['rpm_rad_s']) for p in rotor),default=None)
    goals=[e for e in events if e['event']=='SEGMENT_GOAL']
    drive_directions=[]
    directed_travel=None
    cross_track=None
    primary_gears=[e for e in events if e['event']=='DIRECTION_ARMED_WHILE_STOPPED' and e.get('direction')==segment['direction']]
    if goals:
        start=goals[0]['t']
        motion=timelines[0].window(end,end-start)
        gates=timelines[2].window(end,end-start)
        drive_directions=sorted({int(math.copysign(1,g['v'])) for g in gates if g['mode']=='DRIVE' and abs(g['v'])>.005})
        forward,reverse,_=speed_and_distance(motion)
        directed_travel=forward if segment['direction']>0 else reverse
        if segment['id'] in cfg['connector_qualification']['segments']:
            curve=LineString([p[:2] for p in segment['poses_world']])
            cross_track=max((curve.distance(Point(p['xyz'][:2])) for p in motion),default=None)
    checks = dict(ordered_worker_admission=completed and any(e['event']=='SEGMENT_ARRIVAL' for e in events),
        original_stop_window=held['samples']>=limits['minimum_samples'] and span>=limits['span_s'],
        sampled=sampled(truth,.05,span) and sampled(rotor,.11,span) and sampled(gate,.09,span),
        physical_xy=err_xy is not None and err_xy<=nav['intermediate_xy_tolerance_m'],
        physical_yaw=err_yaw is not None and err_yaw<=nav['intermediate_yaw_tolerance_rad'],
        body_stopped=max_v is not None and max_v<=safety['stopped_v_mps'] and max_w is not None and max_w<=safety['stopped_w_rad_s'],
        blade_stopped=max_rotor is not None and max_rotor<.1 and all(p['healthy'] and not p['on'] for p in rotor),
        gate_stopped=bool(gate) and all(g['tool_stopped'] and g['stopped'] for g in gate))
    if row['gear_changed_at_entry']:
        checks['new_gear_armed_at_physical_stop']=bool(goals) and any(e['t']<=goals[0]['t'] for e in primary_gears)
    if segment['id'] in cfg['connector_qualification']['segments']:
        checks['required_stopped_gear_armed']=bool(goals) and any(e['t']<=goals[0]['t'] for e in primary_gears)
        checks['strict_original_gear_and_route']=(drive_directions==[segment['direction']] and
            directed_travel is not None and directed_travel>.5*segment['length_m'] and
            cross_track is not None and cross_track<=nav['endpoint_recovery']['max_tracking_error_m'])
    return dict(row, measured_drive_directions=drive_directions,directed_travel_m=directed_travel,
                connector_max_cross_track_m=cross_track,state='PHYSICALLY_VERIFIED' if all(checks.values()) else 'PARKING_NOT_CONFIRMED',
                sim_s=end,span_s=span,loc_samples=held['samples'],truth_samples=len(truth),tool_samples=len(rotor),
                gate_samples=len(gate),max_xy_m=err_xy,max_yaw_rad=err_yaw,max_yaw_deg=math.degrees(err_yaw) if err_yaw is not None else None,
                max_linear_mps=max_v,max_angular_rad_s=max_w,max_blade_rad_s=max_rotor,checks=checks)


def fragment_feedback(sections, by_segment, rotor, mission, cfg, min_working_rpm):
    """Associate each blade-on fragment with its own sensor interval and original job."""
    result=[]
    for i,section in enumerate(sections):
        if not section['tool_on']:
            continue
        sid=section['id']
        key=cfg['version']+':'+section['work_key']
        times=[e['t'] for e in by_segment.get(sid,[]) if e['event']=='START_WORK_REQUEST']
        next_entries=[e['t'] for later in sections[i+1:] for e in by_segment.get(later['id'],[])
                      if e['event'] in ('DIRECTION_ARMED_WHILE_STOPPED','SEGMENT_GOAL','START_WORK_REQUEST')]
        stop=min(next_entries,default=mission['t'])
        samples=rotor.window(stop,stop-times[0]) if times and times[0]<=stop else []
        working=[r for r in samples if r['key']==key and r['healthy'] and abs(r['rpm_rad_s'])>=min_working_rpm]
        result.append(dict(segment=sid,original_key=key,worker_completed=sid in mission['completed_segments'],
                    started_at=times[0] if times else None,interval_end=stop if times else None,
                    actual_blade_samples=len(working),measured=bool(working)))
    return result


def bind_session(session, cfg):
    spec=read(session/'spec.json')
    gen=spec.get('generated_bindings',{})
    manager=read(session/'scene/connector_manager.json')
    tool=read(session/'scene/tool.json')
    bt=read(session/'bt.json')
    return (bool(gen) and all((session/p).is_file() and sha(session/p)==h for p,h in gen.items())
            and cfg==read(session/'scene/connector_mission.json') and bt['task_version']==manager['plan_revision']
            ==tool['plan_revision']==cfg['version'] and tool['session']==bt['session']==spec['session']
            and bt['vehicle_hash']==cfg['vehicle']['config_hash'])


def evaluate(session):
    session=Path(session).resolve()
    spec,cfg,cluster=check_scope(session)
    # Retain the original formal analyzer output in this experimental session.
    # It also explicitly compares its plan to the official 47-section edition.
    base=analyze(session)
    mission=read(session/'mission_result.json')
    by_segment=defaultdict(list)
    for e in rows(session/'mission_events.jsonl'):
        if 'segment' in e:
            by_segment[e['segment']].append(e)
    timelines=[Timeline(rows(session/path)) for path in ('evaluation/truth.jsonl','tool_evaluation/physical.jsonl','control/gate.jsonl')]
    sections=cluster['segments']
    stops=[stop_row(s,by_segment.get(s['id'],[]),mission,cfg,timelines,
                    sections[i-1]['direction'] if i else None, sections[i+1]['direction'] if i+1<len(sections) else None)
           for i,s in enumerate(sections)]
    jobs=read(session/'tool_evaluation/result.json')['jobs']
    target=requested_target(cluster,sections,True)
    effect=union_all([shape(v['area']) for v in jobs.values() if v['active_samples']>0])
    physical_cut=effect.intersection(target)
    confirmed=base['selected_confirmed_union_m2']
    checks=dict(base['checks'])
    archived_formal_version_match=checks.pop('task_version_bound')
    checks['candidate_version_bound']=bind_session(session,cfg)
    checks['all_46_in_order']=mission['completed_segments']==[s['id'] for s in sections]
    checks['original_1800s_budget']=mission['t']<=cluster['budget_s']
    checks['every_required_park_physically_verified']=all(s['state']=='PHYSICALLY_VERIFIED' for s in stops)
    checks['active_cut_measured']=physical_cut.area>0 and all(j['active_samples']>0 for k,j in jobs.items()
                                                               if k and any(a['key']==k for a in base['jobs']))
    fragments=fragment_feedback(sections,by_segment,timelines[1],mission,cfg,read(session/'scene/tool.json')['working_min_rad_s'])
    confirmed_jobs={j['key'] for j in base['jobs'] if j['confirmed_m2']>0 and not j['uncertain']}
    checks['measured_feedback_for_each_selected_work_fragment']=bool(fragments) and all(
        f['worker_completed'] and f['measured'] and f['original_key'] in confirmed_jobs for f in fragments)
    checks['feedback_union_and_residual_reconcile']=abs(confirmed+base['selected_residual_m2']-target.area)<1e-6
    proven=[r for r in stops if r['state']=='PHYSICALLY_VERIFIED']
    terminal=mission['terminal']
    result=dict(schema='B1-FULL-46-PHYSICAL/1',status='PASS' if all(checks.values()) else 'PARTIAL',
            session=session.name,plan_revision=cfg['version'],map_revision=cfg['map_revision'],
            full_entrance_selected=True,sim_budget_s=spec['sim_seconds'],sim_elapsed_s=mission['t'],
            runtime_wall_s=read(session/'runtime.json')['wall_seconds'],terminal=terminal,error=mission.get('error'),
            total_segments=len(sections),completed_segments=len(mission['completed_segments']),parking_verified=len(proven),
            gear_change_parks_verified=sum(r['gear_changes_after'] for r in proven),
            max_verified_parking_xy_m=max((r['max_xy_m'] for r in proven),default=None),
            max_verified_parking_yaw_deg=max((r['max_yaw_deg'] for r in proven),default=None),
            actual_cut_m2=physical_cut.area,raw_cut_union_all_ground_m2=effect.area,
            confirmed_union_m2=confirmed,target_m2=target.area,residual_m2=base['selected_residual_m2'],
            physical_uncut_target_m2=target.difference(physical_cut).area,
            area_definition='actual_cut_m2 is independent rotor effect clipped to the original target. confirmed_union_m2 comes from the SQLite ledger. residual_m2 is original target minus confirmed union.',
            work_jobs=base['jobs'],work_fragments=fragments,checks=checks,archived_47_section_version_match=archived_formal_version_match,
            failed_checks=[k for k,v in checks.items() if not v],formal_credit=False,
            original_xy_m=cfg['navigation_execution']['intermediate_xy_tolerance_m'],
            original_yaw_rad=cfg['navigation_execution']['intermediate_yaw_tolerance_rad'],
            parking=stops,evaluator_sha256=sha(Path(__file__)),sources={p:sha(session/p) for p in (
                'spec.json','mission.json','scene/fixture.json','mission_result.json','mission_events.jsonl','evaluation/truth.jsonl',
                'evaluation/contacts.json','tool_evaluation/physical.jsonl','tool_evaluation/result.json','ledger/ledger.sqlite',
                'control/gate.jsonl','runtime.json','analysis_mowing_r10.json')})
    dump(session/'full_46_physical_report.json',result)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path)
    args=parser.parse_args()
    result=evaluate(args.session)
    print(json.dumps({k:result[k] for k in ('status','completed_segments','parking_verified','actual_cut_m2','confirmed_union_m2','residual_m2','failed_checks')},indent=2))
    raise SystemExit(0 if result['status']=='PASS' else 2)
