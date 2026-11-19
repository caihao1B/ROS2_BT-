"""Cold-start B1 at the qualified map entry and physically transfer to the seven late diagnostic segments."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
from tempfile import TemporaryDirectory

import mowing_yaw_candidate_r10 as initial
from common import APP, PKG, canonical, dump, edition, read, sha
from footprint_r09 import body_polygon
from recovery_mowing_r10 import gear_runs, validate_candidate, yaw_error
from recovery_stop_candidate_r10 import RecoveryStopEvidence
from shapely.geometry import shape

PROFILE='verified-entry-transfer'
SCRIPTS=PKG/'scripts'
TRANSFER=APP/'candidates/mowing_verified_entry_transfer.json'
START,COUNT=21,7


def checked_plan():
    plan=read(TRANSFER)
    revision=plan.pop('revision')
    if canonical(plan)!=revision:
        raise ValueError('Transfer edition checksum changed')
    cfg=read(edition('R10')/'mowing/mission.json');safe=read(edition('R10')/'mowing_safe_plan.json')
    route=cfg['clusters'][0];points=plan['points']
    if (plan['task_version']!=cfg['version'] or plan['safe_plan_revision']!=safe['revision']
            or sha(APP/'build/r10_safe_route/r10_safe_route')!=plan['backend_sha256']
            or sha(PKG/'r10_backend/safe_route.cpp')!=plan['backend_source_sha256']):
        raise ValueError('Bound task, safety edition or solver changed')
    start=route['segments'][0]['poses'][0];goal=route['segments'][START]['poses'][0]
    if (plan['start']!=start or plan['goal']!=goal or abs(plan['radius_m']-route['turn_radius_m'])>1e-9
            or math.dist(points[0][:2],start[:2])>1e-6 or math.dist(points[-1][:2],goal[:2])>1e-6
            or yaw_error(points[0][2],start[2])>1e-6 or yaw_error(points[-1][2],goal[2])>1e-6):
        raise ValueError('Declared transfer endpoints must be original approved poses')
    ok,why=validate_candidate(points,start,goal,cfg,35.)
    allowed=shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
    if not ok or any(not allowed.covers(body_polygon(safe['planning_envelope'],p[0]+28,p[1]+14,p[2])) for p in points):
        raise ValueError('Transfer violates geometry or enlarged prior footprint: '+why)
    segments=[]
    for index,run in enumerate(gear_runs(points)):
        poses=run['poses'];length=sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))
        segments.append(dict(id=f'QUALIFICATION_TRANSFER:{index}',kind='TURN_TOOL_OFF',tool_on=False,
                             direction=run['direction'],poses=poses,poses_world=[[x+28,y+14,h] for x,y,h in poses],
                             length_m=length,nominal_seconds=length/.35,row_id=None,source_segment_id=None,
                             target_area=[],requested_area_m2=0.))
    return dict(revision=revision,frozen_file_sha256=sha(TRANSFER),source_sha256=plan['backend_source_sha256'],
                qualified_original_entry=start,selected_start=goal,selected_start_index=START,segments=segments,
                length_m=plan['length_m'],method=plan['method'],dense_samples=len(points),requested_work_m2=0.,
                scope='All connector gears must be driven, separately measured and have no ledger obligation.')


def compose(out,task,session,rtf,cluster,case,segment_start):
    if segment_start!=START or cluster!='B1' or case!='nominal':
        raise ValueError('Undeclared transfer case')
    cfg=initial.candidate_scene(out,task,session,rtf,cluster,case,0)
    transfer=checked_plan()
    dump(out/'qualification_transfer.json',transfer)
    fixture=read(out/'fixture.json')
    fixture.update(session_profile=PROFILE,transfer_sha256=sha(out/'qualification_transfer.json'),
                   scope='Cold start at the unchanged accepted B1 physical spawn and coarse operator hint; '
                         'physically travel with tool off to the late seven original segments.')
    dump(out/'fixture.json',fixture)
    return cfg


def overlay(task,out):
    adapt=initial.overlay(task,out);generator=adapt/'adapter.py'
    marker="angular[2]+=f.get('gyro_z_bias_rad_s',0)"
    generated=initial.replace_once(generator.read_text(),marker,marker+";angular[2]-=.005*min(1.,max(0.,(t-60.)/3.))")
    compile(generated,str(generator),'exec');generator.write_text(generated)
    manifest=read(adapt/'adaptation.json');manifest['generated']['adapter.py']=sha(generator)
    manifest['test_sensor_profile']=dict(start_s=60.,ramp_s=3.,added_yaw_bias_rad_s=-.005,scope='SIMULATED_SENSOR_ONLY')
    dump(adapt/'adaptation.json',manifest)
    return adapt


def worker_source():
    source=initial.worker_source()
    source=initial.replace_once(source,"spec.get('candidate_profile')=='late-warm'", "spec.get('candidate_profile')=='verified-entry-transfer'")
    source=initial.replace_once(source,"max(45.,begin+15.),45","max(90.,begin+18.),95")
    marker="segments=execution_groups(selected,cfg['navigation_execution']['mode'])"
    inserted="""transfer_path=out/'scene/qualification_transfer.json'
 if hashlib.sha256(transfer_path.read_bytes()).hexdigest()!=json.loads((out/'scene/fixture.json').read_text())['transfer_sha256']:raise RuntimeError('TRANSFER_EDITION_CHANGED')
 transfer=json.loads(transfer_path.read_text())['segments'];transfer_ids={seg['id'] for seg in transfer};transfer_passages=[]
 segments=execution_groups(transfer+selected,cfg['navigation_execution']['mode'])"""
    source=initial.replace_once(source,marker,inserted)
    source=initial.replace_once(source,'passages.append(record)',"(transfer_passages if member['id'] in transfer_ids else passages).append(record)")
    source=initial.replace_once(source,"log('ORIGINAL_SEGMENT_PASSAGE',segment=member", "log('QUALIFICATION_TRANSFER_PASSAGE' if member['id'] in transfer_ids else 'ORIGINAL_SEGMENT_PASSAGE',segment=member")
    marker="completed.extend(m['id'] for m in s['members'])"
    if source.count(marker)!=2:raise ValueError('Changed completion paths')
    source=source.replace(marker,"completed.extend(m['id'] for m in s['members'] if m['id'] not in transfer_ids)")
    source=initial.replace_once(source,"passed=terminal=='COMPLETED' and len(completed)==len(selected)",
                                "passed=terminal=='COMPLETED' and len(completed)==len(selected) and len(transfer_passages)==len(transfer_ids)")
    source=initial.replace_once(source,'original_segment_passages=passages,','qualification_transfer_passages=transfer_passages,original_segment_passages=passages,')
    marker="    if s['kind']=='COVERAGE' and not st['work_complete'] and not begin_tool(s):"
    precheck="""    if s['id'] in transfer_ids:
     wait(pathcheck.service_is_ready,5);query=IsPathValid.Request();query.path.header.frame_id='map';query.path.header.stamp=n.get_clock().now().to_msg();query.path.poses=[pose(p) for p in s['poses']]
     checked=result(pathcheck.call_async(query),5)
     log('QUALIFICATION_TRANSFER_COSTMAP_CHECK',segment=s['id'],valid=checked.is_valid,direction=s['direction'],samples=len(s['poses']))
     if not checked.is_valid:raise RuntimeError('QUALIFICATION_TRANSFER_LIVE_COSTMAP_BLOCKED')
"""
    return initial.replace_once(source,marker,precheck+marker)


def runner():
    # Reuse the isolated candidate lifecycle and all frozen ownership/teardown logic.
    run=initial.runtime(PROFILE)
    run.__globals__.update(__file__=__file__,compose=compose,overlay=overlay,
                           candidate_sources=initial.CANDIDATE_SOURCES+('mowing_verified_transfer_r10.py',),
                           worker_hash=hashlib.sha256(worker_source().encode()).hexdigest())
    return run


def verify(run):
    from analyze_mowing_r10 import analyze
    from analyze_r02 import rows
    result=analyze(run)
    spec=read(run/'spec.json');job=read(run/'mission_result.json')
    events=rows(run/'mission_events.jsonl');gates=rows(run/'control/gate.jsonl')
    physics=rows(run/'tool_evaluation/physical.jsonl');inputs=rows(run/'inputs/inputs.jsonl')
    tr=read(run/'scene/qualification_transfer.json')
    ids=[s['id'] for s in tr['segments']]
    prefix=[r for r in events if r['event']=='QUALIFICATION_TRANSFER_PASSAGE']
    accepted=[r for r in events if r['event']=='QUALIFICATION_TRANSFER_COSTMAP_CHECK' and r['valid']]
    start=next((e['t'] for e in events if e['event']=='WARM_TEST_STOP_BEGIN'),None)
    end=next((e['t'] for e in events if e['event']=='WARM_TEST_STOP_END'),None)
    terminal=job.get('t',0.)
    hold=[p for p in physics if start is not None and end is not None and start<=p['t']<=end]
    gates_at_hold=[g for g in gates if start is not None and end is not None and start<=g['t']<=end]
    span=end-start if end is not None and start is not None else None
    entry=next((e['t'] for e in events if e['event']=='SEGMENT_GOAL' and e['segment'] in ids),None)
    arrived=prefix[-1]['t'] if prefix and prefix[-1]['segment']==ids[-1] else None
    transfer_physical=[p for p in physics if end is not None and end<=p['t']<=(arrived if arrived is not None else terminal)]
    rate=[r['gyro'][2] for r in inputs if r['key']=='imu' and end is not None and end-5.<=r['t']<=end]
    wheel=[r['w'] for r in inputs if r['key']=='wheel' and end is not None and end-5.<=r['t']<=end]
    drift=statistics.mean(rate)-statistics.mean(wheel) if rate and wheel else None
    stop_spd=read(run/'mission.json')['vehicle']['safety']['stopped_v_mps']
    high_speed=max((abs(r['body_v']) for r in hold),default=None)
    confirmed=[e for e in events if e['event']=='RECOVERY_STOP_FEEDBACK_CONFIRMED']
    originals=[x['segment'] for x in job.get('original_segment_passages',[])]
    checks=dict(baseline_physical_checks=result['passed'],
                generated_files_bound=all(sha(run/p)==h for p,h in spec['candidate_generated_sha256'].items()),
                effective_worker_bound=spec['candidate_worker_sha256']==hashlib.sha256(worker_source().encode()).hexdigest(),
                transfer_edition_bound=tr['frozen_file_sha256']==sha(TRANSFER) and read(run/'scene/fixture.json')['transfer_sha256']==sha(run/'scene/qualification_transfer.json'),
                physical_warm_hold=bool(span is not None and span>=18 and end>=90 and hold and gates_at_hold and high_speed<stop_spd
                                        and all(abs(r['rpm_rad_s'])<.1 for r in hold) and all(g['mode']=='STOP' and g['stopped'] for g in gates_at_hold)),
                warm_gyro_consistent=drift is not None and abs(drift)<.0015,
                transfer_order_measured=[e['segment'] for e in prefix]==ids,
                each_transfer_gear_checked=all(any(e['segment']==sid for e in accepted) for sid in ids),
                transfer_physical_tool_off=bool(transfer_physical) and all(r['flow']==0. and abs(r['rpm_rad_s'])<.1 for r in transfer_physical),
                no_transfer_ledger_credit=not any('QUALIFICATION_TRANSFER' in str(j['key']) for j in result['jobs']),
                original_problem_turn_measured='B1:safe_turn:6:3' in originals,
                new_feedback_barrier_exercised=bool(confirmed) and all(e['t']-e['requested_at']>=e['minimum_span_s'] for e in confirmed))
    report=dict(passed=all(checks.values()),checks=checks,original_checks=result['checks'],profile=PROFILE,
                transfer=dict(requested_work_m2=0.,planned_length_m=tr['length_m'],planned_gears=ids,
                              confirmed_gears=[r['segment'] for r in prefix],first_command_sim_s=entry,
                              last_original_entry_arrival_sim_s=arrived,travel_elapsed_s=(arrived-entry if arrived is not None and entry is not None else None),
                              tool_feedback_samples=len(transfer_physical)),
                warm_hold=dict(start=start,end=end,elapsed_s=span,original_stop_gate_mps=stop_spd,
                               max_physical_speed_mps=high_speed,gyro_minus_encoder_rad_s=drift),
                original_selected=dict(requested=COUNT,passages=originals,remaining=job.get('remaining_selected'),
                                       confirmed_area_m2=result['selected_confirmed_union_m2'],residual_area_m2=result['selected_residual_m2']),
                recovery_stop_feedback_count=len(confirmed),adapter=read(run/'inputs/result.json'),
                terminal=job['terminal'],terminal_error=job['error'],elapsed_sim_s=terminal,formal_credit=False)
    dump(run/'candidate_analysis.json',report);return report


if __name__=='__main__':
    if sys.argv[1:2]==['worker']:
        del sys.argv[1]
        exec(compile(worker_source(),str(SCRIPTS/'mission_mowing_r10.py'),'exec'),
             dict(__name__='__main__',__file__=str(SCRIPTS/'mission_mowing_r10.py'),hashlib=hashlib,RecoveryStopEvidence=RecoveryStopEvidence))
    else:
        parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--prepare',action='store_true')
        parser.add_argument('--verify',type=Path);parser.add_argument('--rtf',type=float,default=.4)
        args=parser.parse_args()
        if args.prepare:
            with TemporaryDirectory(prefix='mow_transfer_') as temp:
                out=Path(temp);compose(out/'scene','mowing','prepare',args.rtf,'B1','nominal',START);overlay('mowing',out/'overlay')
                runner();compile(worker_source(),str(SCRIPTS/'mission_mowing_r10.py'),'exec')
                plan=read(out/'scene/qualification_transfer.json')
                print(json.dumps({k:v for k,v in plan.items() if k!='segments'}));print('Checked gears',[(s['direction'],len(s['poses'])) for s in plan['segments']])
        else:
            path=args.verify or runner()(task='mowing',seconds=780,rtf=args.rtf,cluster='B1',segment_limit=COUNT,case='nominal',segment_start=START)
            report=verify(path);print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 2)
