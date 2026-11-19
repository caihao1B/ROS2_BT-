"""Export compact, independently remeasurable area and stop evidence from a full B1 attempt."""
import argparse
from collections import Counter
import gzip
import json
import math
from pathlib import Path
import shutil
import sqlite3
import statistics
import subprocess

from shapely import from_wkb, union_all
from shapely.geometry import shape, mapping
from common import dump, read, sha
from safe_plan_r10 import requested_target
from score_b1_connector import rows, pose_error, speed_and_distance
from score_b1_full import check_scope, Timeline


def replay_area(bundle):
    value=json.loads(gzip.decompress(bundle.read_bytes()))
    target=shape(value['target_map'])
    effect=union_all([shape(j['area']) for j in value['rotor_evaluator_jobs'].values() if j['active_samples']>0])
    credit=union_all([from_wkb(bytes.fromhex(j['wkb_hex'])) for j in value['sqlite_jobs'] if j['wkb_hex']])
    return dict(target_m2=target.area,raw_cut_union_all_ground_m2=effect.area,actual_cut_m2=effect.intersection(target).area,
                confirmed_union_m2=credit.intersection(target).area,residual_m2=target.difference(credit).area)


def audit(session,destination,video=None):
    session=session.resolve()
    spec,cfg,cluster=check_scope(session)
    report=read(session/'full_46_physical_report.json')
    destination.mkdir(exist_ok=True,parents=True)
    mission=read(session/'mission_result.json')
    events=rows(session/'mission_events.jsonl')
    risks=rows(session/'risk/risk.jsonl')
    contexts=rows(session/'decisions.jsonl')
    interruptions=[]
    for e in events:
        if e['event'] not in ('BT_INTERRUPTS_SEGMENT','BT_INTERRUPTS_START_WORK'):
            continue
        # The control context must precede the worker event. The supervisor and worker
        # update their local /clock callbacks separately (up to 30 ms scheduling skew).
        decisions=[d for d in contexts if abs(d['t']-e['t'])<=.03 and 0<=e['t']-d['evidence']['stamp']<.15 and
                   d['action']==e['action'] and d['reason_code']==e['reason']]
        if not decisions:
            raise ValueError('No matching fresh BT event for '+str(e))
        decision=min(decisions,key=lambda d:abs(d['t']-e['t']))
        ev=decision['evidence']
        interruptions.append(dict(event=e,decision_stamp=decision['t'],dispatch_id=decision['dispatch_id'],
            sequence=ev['sequence'],context_stamp=ev['stamp'],valid_localization=ev['localized'],
            blocked=ev['blocked'],sensors_healthy=ev['sensors_healthy'],safety_grid_age_s=ev['sensor_age'],
            terrain_capture_s=ev.get('sensor_capture'),progress_m=ev['progress_m']))
    t=mission['t']
    ground=Timeline(rows(session/'evaluation/truth.jsonl'))
    rotor=Timeline(rows(session/'tool_evaluation/physical.jsonl'))
    poses=ground.window(t,.6)
    physical=rotor.window(t,.6)
    if not physical or not poses:
        raise ValueError('Missing terminal independent physical data')
    end_pose=poses[-1]
    last=next((x for x in reversed(events) if x['event']=='SEGMENT_GOAL'),None)
    goal=next((s for s in cluster['segments'] if last and s['id']==last['segment']),None)
    _,_,w=speed_and_distance(poses)
    stopped_window=rotor.window(t+.11,.21)
    err=pose_error(end_pose['xyz'],end_pose['rpy'][2],goal['poses_world'][-1]) if goal else (None,None)
    phase=[r for r in risks if last and r['t']>=next(e['t'] for e in events if e['event']=='SEGMENT_GOAL') and r['t']<=t]
    native=[r['terrain_health'] for r in phase if r.get('terrain_health') and 'compute_ms' in r['terrain_health']]
    q=read(session/'evaluation/contacts.json')
    target=requested_target(cluster,cluster['segments'],True)
    physical_jobs=read(session/'tool_evaluation/result.json')['jobs']
    with sqlite3.connect('file:'+str(session/'ledger/ledger.sqlite')+'?mode=ro',uri=True) as conn:
        jobs=[dict(key=key,status=status,quantity=quantity,uncertain=bool(uncertain),wkb_hex=bytes(wkb).hex() if wkb else '')
              for key,status,quantity,uncertain,wkb in conn.execute('SELECT key,status,quantity,uncertain,credit FROM jobs ORDER BY key')]
    bundle=dict(schema='B1-RAW-AREA-EVIDENCE/1',session=session.name,plan_revision=cfg['version'],
                original_area_map_m2=cluster['work_area_m2'],target_map=mapping(target),
                rotor_evaluator_jobs=physical_jobs,sqlite_jobs=jobs,physical_contacts=q,
                terminal_physical_body_stream=poses,terminal_physical_rotor_stream=physical,
                original_stop_goal_world=goal['poses_world'][-1] if goal else None,
                terminal_coverage_obligation=goal['id'] if goal else None,
                source_hashes={p:sha(session/p) for p in ['tool_evaluation/result.json','ledger/ledger.sqlite','mission.json','evaluation/contacts.json',
                                                          'risk/risk.jsonl','decisions.jsonl','mission_events.jsonl']})
    archive=destination/'area_and_stop_observations.json.gz'
    archive.write_bytes(gzip.compress(json.dumps(bundle,separators=(',',':'),allow_nan=False).encode(),mtime=0))
    areas=replay_area(archive)
    if any(abs(report[k]-v)>1e-8 for k,v in areas.items()):
        raise ValueError('SQLite/independent measurements did not reproduce the physical report')
    summary=dict(schema='B1-ENTRANCE-FAULT-AND-AREA/1',session=session.name,plan_revision=cfg['version'],
      status=report['status'],terminal=mission['terminal'],error=mission.get('error'),sim_s=t,real_s=report['runtime_wall_s'],
      complete_sections=report['completed_segments'],selected_sections=len(cluster['segments']),areas=areas,
      stop=dict(world_xyz=end_pose['xyz'],measured_yaw_rad=end_pose['rpy'][2],truth_time=end_pose['t'],
                yaw_rad_max_rate_last_600ms=w,max_observed_body_speed_last_600ms_mps=max(abs(r['body_v']) for r in physical),
                max_rotor_rad_s_last_600ms=max(abs(r['rpm_rad_s']) for r in physical),all_rotors_off_last_600ms=all(not r['on'] for r in physical),
                last_attempted_endpoint=goal['id'] if goal else None,physical_distance_to_attempted_endpoint_m=err[0],
                physical_yaw_error_to_attempted_endpoint_rad=err[1],final_body_speed_max_mps=max((abs(r['body_v']) for r in stopped_window),default=None),
                final_rotor_max_rad_s=max((abs(r['rpm_rad_s']) for r in stopped_window),default=None),
                final_window_s=[t-.1,t+.11],note='Terminal safety stop. A stopped partial segment receives no endpoint or gear qualification.'),
      interruptions=interruptions,perception=dict(original_gate_s=.45,risk_frames_during_work=len(phase),
                risk_reasons=dict(Counter(r['reason'] for r in phase)),
                native_snapshot_count=len(native),native_reported_processing_ms_median=statistics.median(r['compute_ms'] for r in native) if native else None,
                native_reported_processing_ms_max=max((r['compute_ms'] for r in native),default=None),
                note='Risk timestamps are capture time. Age at the BT context comes from the gate reception, including upstream processing and transport.'),
      archive_sha256=sha(archive),clean_processes=read(session/'runtime.json')['passed'],
      rotor_area_sha256=sha(session/'tool_evaluation/result.json'),final_stop_evidence=report['checks']['measured_final_stop'],
      no_unexpected_contact=report['checks']['no_unexpected_contact'],formal_credit=False)
    if video:
        import imageio_ffmpeg
        import re
        raw=session/'video/full.mp4'
        args=[imageio_ffmpeg.get_ffmpeg_exe(),'-v','error','-nostdin','-progress','pipe:1','-i',str(raw),'-map','0:v:0','-f','null','-']
        process=subprocess.run(args,capture_output=True,text=True,check=True,timeout=90)
        numbers=re.findall(r'^frame=(\d+)$',process.stdout,flags=re.MULTILINE)
        video_info=read(session/'video/recording.json')
        if not numbers or int(numbers[-1])!=video_info['frames'] or not video_info.get('no_generated_motion') or not video_info['valid']:
            raise ValueError('Full recording frame QA failed')
        video.parent.mkdir(parents=True,exist_ok=True)
        if video.exists() and sha(video)!=sha(raw):
            raise ValueError('Output already contains another recording')
        if not video.exists():
            shutil.copyfile(raw,video)
        summary['video']=dict(file=video.name,sha256=sha(video),bytes=video.stat().st_size,decoded_frames=int(numbers[-1]),
                              duration_s=video_info['duration_s'],source=video_info['source'],fps=video_info['fps'])
    dump(destination/'entrance_observations.json',summary)
    shutil.copyfile(session/'full_46_physical_report.json',destination/'full_46_physical_report.json')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path)
    parser.add_argument('--export',type=Path)
    parser.add_argument('--movie',type=Path)
    parser.add_argument('--replay-area',type=Path)
    args=parser.parse_args()
    if args.replay_area:
        print(json.dumps(replay_area(args.replay_area),indent=2))
    elif args.session and args.export:
        value=audit(args.session,args.export,args.movie)
        print(json.dumps({k:value[k] for k in ['status','areas','stop','perception','video'] if k in value},indent=2))
    else:
        parser.error('provide --replay-area or --session and --export')
