"""Evaluate an isolated B1 physical turn offline. Never publishes or supplies a vehicle pose."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
from shapely.geometry import LineString, Point, shape

from common import ROOT, canonical, dump, read, sha
from footprint_r09 import body_polygon


def load_json(path):
    return read(path) if path.is_file() else {}


def rows(path):
    if not path.is_file():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def yaw_error(a, b):
    return abs(math.remainder(a-b, math.tau))


def between(records, begin, end):
    return [r for r in records if begin-1e-6 <= r['t'] <= end+1e-6]


def pose_error(world, yaw, goal):
    return math.dist(world[:2], goal[:2]), yaw_error(yaw, goal[2])


def terminal_offsets(pose, goal, gear):
    """Goal frame +x is forward along the final vehicle heading, +y to its left."""
    dx, dy = pose[0]-goal[0], pose[1]-goal[1]
    h = goal[2]
    longitudinal = math.cos(h)*dx + math.sin(h)*dy
    return dict(longitudinal_m=longitudinal, lateral_left_m=-math.sin(h)*dx+math.cos(h)*dy,
                overshoot_in_motion_direction_m=gear*longitudinal,
                yaw_error_signed_rad=math.remainder(pose[2]-h,math.tau),
                xy_error_m=math.hypot(dx,dy))


def speed_and_distance(truth):
    signed = []
    rates = []
    for a,b in zip(truth, truth[1:]):
        ds = np.array(b['xyz'][:2]) - a['xyz'][:2]
        signed.append(float(ds @ [math.cos(a['rpy'][2]), math.sin(a['rpy'][2])]))
        dt = b['t'] - a['t']
        if dt > 0:
            rates.append(yaw_error(b['rpy'][2],a['rpy'][2])/dt)
    return sum(max(0,x) for x in signed), sum(max(0,-x) for x in signed), max(rates,default=None)


def sampled(records, max_gap, span):
    times = [r['t'] for r in records]
    return bool(len(times)>=2 and times[-1]-times[0]>=span-max_gap and
                all(0<b-a<=max_gap for a,b in zip(times,times[1:])))


def evaluate(out):
    spec = read(out/'spec.json')
    cfg = read(out/'mission.json')
    mission = load_json(out/'mission_result.json')
    runtime = load_json(out/'runtime.json')
    geometry = read(out/'scene/connector_geometry.json')
    contacts = load_json(out/'evaluation/contacts.json')
    bad = {k:v for k,v in contacts.items() if 'vehicle::' in k and not ('wheel::' in k and 'ground::body::ground' in k)}
    events = rows(out/'mission_events.jsonl')
    truth = rows(out/'evaluation/truth.jsonl')
    blade = rows(out/'tool_evaluation/physical.jsonl')
    gates = rows(out/'control/gate.jsonl')
    refs = rows(out/'reference/reference.jsonl')
    valid_refs = [r for r in refs if r.get('valid') and 'pose' in r]
    quality = rows(out/'quality/evidence.jsonl')
    trace = [r for r in quality if r.get('valid')]
    authority = shape(cfg['allowed_prior']).buffer(-cfg['footprint_margin_m'])
    safe = cfg['vehicle']['safety']
    xy = cfg['navigation_execution']['intermediate_xy_tolerance_m']
    yaw = cfg['navigation_execution']['intermediate_yaw_tolerance_rad']
    limits = cfg['navigation_execution']['stopped_arrival_confirmation']
    max_track = cfg['navigation_execution']['endpoint_recovery']['max_tracking_error_m']
    cluster = next(c for c in cfg['clusters'] if c['cluster']=='B1')
    sections = cluster['segments'][spec['segment_start']:spec['segment_start']+spec['segment_limit']]
    outcomes = []
    parking_samples=[]
    for k,s in enumerate(sections):
        sid = s['id']
        goal = s['poses_world'][-1]
        action = [e for e in events if e['event']=='SEGMENT_GOAL' and e['segment']==sid]
        admissions = [e for e in events if e['event']=='STOPPED_ORIGINAL_ARRIVAL_CONFIRMED' and e['segment']==sid]
        accepted = [e for e in events if e['event']=='SEGMENT_ARRIVAL' and e['segment']==sid]
        terminal = [e for e in events if e['event']=='TASK_STOP']
        row = dict(segment=sid,direction=s['direction'],length_m=s['length_m'],goal_map=s['poses'][-1],
            action_accepted=len(action),state='NOT_STARTED',parking_window=None,stop_observation=None)
        if action:
            state = 'ATTEMPTED'
            success = bool(accepted and admissions and sid in mission.get('completed_segments',[]))
            final_stop = next((e for e in reversed(events) if terminal and e['event']=='BOTH_PHYSICAL_STOPS_CONFIRMED' and e['t']>=terminal[-1]['t']),None)
            event = admissions[-1] if success else final_stop or (terminal[-1] if terminal else None)
            end = event['t'] if event else mission.get('t',truth[-1]['t'] if truth else action[-1]['t'])
            motion = between(truth,action[0]['t'],end)
            dg = between(gates,action[0]['t'],end)
            command = [g for g in dg if g['mode']=='DRIVE' and abs(g['v'])>.005]
            line = LineString([p[:2] for p in s['poses_world']])
            deviation = max((line.distance(Point(p['xyz'][:2])) for p in motion),default=None)
            forward,reverse,_ = speed_and_distance(motion)
            travel = forward if s['direction']>0 else reverse
            drive_direction = {int(math.copysign(1,g['v'])) for g in command}
            remaining = [e for e in events if e['event']=='DIRECTION_ARMED_WHILE_STOPPED' and e['segment']==sid and e['t']<=action[0]['t']]
            row.update(first_goal_sim_s=action[0]['t'], stopped_gear_armed=bool(remaining), physical_forward_m=forward,
                       physical_reverse_m=reverse, maximum_path_cross_track_m=deviation, completed_by_worker=success)
            if truth and event:
                stop = min(truth,key=lambda r:abs(r['t']-end))
                sx,sh=pose_error(stop['xyz'],stop['rpy'][2],goal)
                physical_offsets=terminal_offsets([*stop['xyz'][:2],stop['rpy'][2]],goal,s['direction'])
                fresh=[r for r in valid_refs if abs(r['t']-end)<=.15]
                stop_ref=min(fresh,key=lambda r:abs(r['t']-end)) if fresh else None
                estimate_offsets=terminal_offsets(stop_ref['pose'],s['poses'][-1],s['direction']) if stop_ref else None
                row['stop_observation']=dict(event=event['event'],sim_s=stop['t'],physical_xy_error_m=sx,physical_yaw_error_rad=sh,
                     physical=physical_offsets,estimated=estimate_offsets,estimate_stamp=stop_ref['t'] if stop_ref else None,
                     cause='POSITION_OUTSIDE' if sx>xy else 'HEADING_OUTSIDE' if sh>yaw else 'INSIDE_PHYSICAL_ENVELOPE')
            if success:
                span = admissions[-1]['span_s']
                end = admissions[-1]['t']
                poses = between(truth,end-span,end)
                tools = between(blade,end-span,end)
                stop_gates = between(gates,end-span,end)
                es = [pose_error(p['xyz'],p['rpy'][2],goal) for p in poses]
                _,_,ang_speed = speed_and_distance(poses)
                max_body = max((abs(p['body_v']) for p in tools),default=None)
                max_rotor = max((abs(p['rpm_rad_s']) for p in tools),default=None)
                data = dict(start_s=end-span,end_s=end,loc_samples=admissions[-1]['samples'],
                       physical_truth_samples=len(poses), physical_rotor_samples=len(tools), span_s=span,
                       estimated_end_xy_m=admissions[-1]['xy_error_m'], estimated_end_yaw_rad=admissions[-1]['yaw_error_rad'],
                       max_physical_xy_m=max((a for a,b in es),default=None),max_physical_yaw_rad=max((b for a,b in es),default=None),
                       max_physical_linear_speed_mps=max_body,max_physical_angular_speed_rad_s=ang_speed,max_rotor_rad_s=max_rotor)
                checks = dict(fresh_held_estimate=data['loc_samples']>=limits['minimum_samples'] and span>=limits['span_s'],
                              gated_stop=sampled(stop_gates,.09,span) and all(g['stopped'] and g['tool_stopped'] for g in stop_gates),
                              independent_measured_hold=sampled(poses,.05,span) and sampled(tools,.11,span),
                              physical_position=bool(es) and data['max_physical_xy_m']<=xy,
                              physical_heading=bool(es) and data['max_physical_yaw_rad']<=yaw,
                              physically_stopped=max_body is not None and max_body<=safe['stopped_v_mps'] and ang_speed is not None and ang_speed<=safe['stopped_w_rad_s'],
                              physical_tool_off=max_rotor is not None and max_rotor<.1 and all(not t['on'] for t in tools),
                              measured_travel_in_selected_gear=travel>.5*s['length_m'] and drive_direction=={s['direction']},
                              route_tracked=deviation is not None and deviation<=max_track,
                              armed=bool(remaining))
                row['parking_window']=dict(**data,checks=checks)
                state='PHYSICALLY_VERIFIED' if all(checks.values()) else 'PARKING_OR_TRACKING_REJECTED'
            row['state']=state
            if state != 'PHYSICALLY_VERIFIED':
                cause=(row.get('stop_observation') or {}).get('cause')
                row['failure_class']=cause if cause and cause!='INSIDE_PHYSICAL_ENVELOPE' else 'INSUFFICIENT_ORDERED_STOP_EVIDENCE' if not success else [key for key,ok in checks.items() if not ok]
        sample=dict(segment=sid,goal_map=s['poses'][-1],state=row['state'],direction=s['direction'],truth_map=[],estimated_map=[],rotor=[],actuator_gate=[])
        if row.get('stop_observation'):
            end=row['stop_observation']['sim_s']
            begin=(row.get('parking_window') or {}).get('start_s',end-.6)
            sample['selection']='stopped admission window' if row['parking_window'] else 'terminal observation only; no credit'
            for p in between(truth,begin,end):
                sample['truth_map'].append([p['t'],p['xyz'][0]-cfg['map_offset'][0],p['xyz'][1]-cfg['map_offset'][1],p['rpy'][2]])
            for p in between(valid_refs,begin,end):
                sample['estimated_map'].append([p['t'],p['odom_t'],*p['pose'][:3]])
            sample['rotor']=[[p['t'],p['body_v'],p['rpm_rad_s'],p['on']] for p in between(blade,begin,end)]
            sample['actuator_gate']=[[g['t'],g['stopped'],g['tool_stopped'],g['v'],g['w']] for g in between(gates,begin,end)]
        parking_samples.append(sample)
        outcomes.append(row)
    ledger = out/'ledger/ledger.sqlite'
    job_count = None
    if ledger.exists():
        with sqlite3.connect('file:'+str(ledger)+'?mode=ro',uri=True) as conn:
            job_count=conn.execute('SELECT count(*) FROM jobs').fetchone()[0]
    original=read(ROOT/'optimization_v2/three_tasks/editions/R10/mowing/mission.json')
    sources = spec['implementation_bindings']
    invalid_bindings=[p for p,h in sources.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    gen=spec.get('generated_bindings',{})
    generated_ok=bool(gen) and all((out/p).is_file() and sha(out/p)==h for p,h in gen.items())
    task_version_ok=(cfg['version']==canonical({k:v for k,v in cfg.items() if k!='version'}) and
         cfg['connector_qualification']['parent_mission_version']==original['version'] and cfg['version']==read(out/'bt.json')['task_version']
         ==read(out/'scene/tool.json')['plan_revision']==read(out/'scene/connector_manager.json')['plan_revision'])
    t=np.array([x['t'] for x in truth])
    pos=np.array([x['xyz'][:2] for x in truth])
    errors = [math.dist(np.add(r['pose'][:2],cfg['map_offset']), [np.interp(r['odom_t'],t,pos[:,j]) for j in (0,1)])
              for r in valid_refs if len(t)>1 and t[0]<=r['odom_t']<=t[-1]]
    rms = math.sqrt(sum(e*e for e in errors)/len(errors)) if errors else None
    admitted = sum(r['state']=='PHYSICALLY_VERIFIED' for r in outcomes)
    own = mission.get('ownership',{})
    outside = [r['t'] for r in truth if not authority.covers(body_polygon(cfg['vehicle']['footprint'],*r['xyz'][:2],r['rpy'][2]))]
    final=mission.get('final_gate') or {}
    motor_target=cfg['vehicle']['steering']['max_curvature']
    commands=[g for g in gates if g['mode']=='DRIVE']
    # The evaluating domain keeps sampling through Nav2 teardown. Inspect the fresh end of the
    # actual admitted task stop and end of physical stream; never infer standstill from zero command.
    final_time=mission.get('t',0.)
    final_blade=between(blade,final_time-.1,final_time+.15)
    final_truth=between(truth,final_time-.1,final_time+.15)
    _,_,final_w=speed_and_distance(final_truth)
    final_velocity=max((abs(x['body_v']) for x in final_blade),default=None)
    final_rpm=max((abs(x['rpm_rad_s']) for x in final_blade),default=None)
    checked_end=bool(final.get('stopped') and final.get('tool_stopped') and final_blade and
                     final_velocity<=safe['stopped_v_mps'] and final_w is not None and final_w<=safe['stopped_w_rad_s'] and final_rpm<.1 and
                     all(not p['on'] for p in final_blade) and blade and abs(blade[-1]['rpm_rad_s'])<.1)
    checks = dict(four_measured_parking_positions_and_headings=admitted==4, four_goals_ordered=mission.get('completed_segments')==[s['id'] for s in sections],
                sensor_localized=bool(trace) and bool(valid_refs) and all(r.get('truth_used') is False for r in trace+valid_refs),
                map_xy_rmse_under_15cm=rms is not None and rms<.15,
                vehicle_footprint_inside_original_authority=bool(truth) and not outside,
                no_unexpected_contact=(out/'evaluation/contacts.json').is_file() and not bad, physical_blades_off_everywhere=bool(blade) and
                all(p['healthy'] and not p['on'] and abs(p['rpm_rad_s'])<.1 for p in blade),zero_work_ledger=job_count==0,
                unique_actuator=own.get('/s03/actuator/request')==['r09_actuator_gate'],
                unique_tool=own.get('/s10/tool/command')==['s10_tool_action_server'],
                unique_tf=set(own.get('/tf',[]))=={'s05_local_odometry','s08_global_pose_manager'} and len(own.get('/tf',[]))==2,
                curvature_bounded=bool(commands) and all(abs(g['w'])<=abs(g['v'])*motor_target+1e-6 for g in commands),
                measured_final_stop=checked_end,
                runtime_clean=runtime.get('passed') is True, original_database_unchanged=runtime.get('original_db_unchanged') is True,
                unchanged_geometry_checks=geometry['passed'], plan_bindings=task_version_ok and generated_ok and not invalid_bindings)
    result=dict(schema='B1-FOUR-GEAR-PHYSICAL/1',passed=all(checks.values()),formal_credit=False,checks=checks,
                session=out.name,relative_session=str(out.relative_to(ROOT)) if out.is_relative_to(ROOT) else str(out),
                plan_revision=cfg['version'],geometry_sha256=geometry['candidate_sha256'],map_revision=cfg['map_revision'],
                selected_sections=4,verified_parking_stops=admitted,trajectory=outcomes,terminal=mission.get('terminal'),terminal_error=mission.get('error'),
                accepted_quality_samples=len(trace),localization_reasons=dict(Counter(r['reason'] for r in quality)),map_xy_rmse_m=rms,
                footprint_outside_samples=len(outside),unexpected_contacts=bad,ledger_jobs=job_count,source_binding_mismatches=invalid_bindings,
                runtime_wall_s=runtime.get('wall_seconds'),runtime_error=runtime.get('error'),forced=runtime.get('forced'),sim_s=mission.get('t'),
                final_stop=dict(task_sim_s=final_time,independent_linear_speed_mps=final_velocity,independent_angular_speed_rad_s=final_w,
                                independent_rotor_rad_s=final_rpm,vehicle_gate=final.get('stopped'),tool_gate=final.get('tool_stopped')),
                physical_limits=dict(position_m=xy,yaw_rad=yaw,stopped_linear_mps=safe['stopped_v_mps'],stopped_angular_rad_s=safe['stopped_w_rad_s'],
                                     tracking_m=max_track,minimum_stopped_s=limits['span_s']),
                scope='Only four tool-off B1 connector sections. Endpoint estimates and isolated Gazebo pose streams are compared on each stopped interval; no cluster or work credit.')
    dump(out/'parking_samples.json',dict(session=out.name,world_to_map_offset=cfg['map_offset'],
         truth_columns=['sim_s','map_x_m','map_y_m','yaw_rad'],estimate_columns=['published_s','odom_sim_s','map_x_m','map_y_m','yaw_rad'],
         tool_columns=['sim_s','body_speed_mps','rpm_rad_s','on'],gate_columns=['sim_s','stopped','tool_stopped','v','w'],sections=parking_samples))
    result['parking_sample_file_sha256']=sha(out/'parking_samples.json')
    dump(out/'physical_connector_report.json',result)
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path)
    report=evaluate(ap.parse_args().session.resolve())
    print(json.dumps(report,indent=2,allow_nan=False))
    raise SystemExit(0 if report['passed'] else 2)
