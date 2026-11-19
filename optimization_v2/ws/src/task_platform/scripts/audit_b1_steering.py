"""Compare the actual Dino geometry and measured steering transfer against the planned qualification."""
import argparse
import json
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml
from common import APP, ROOT, canonical, dump, edition, read, sha


def audit(out):
    cfg=read(out/'mission.json')
    car=read(out/'scene/vehicle.json')
    parent=read(edition('R10')/'mowing/mission.json')['vehicle']
    fields=dict(car)
    fields.pop('config_hash')
    fields.pop('alignment_parent_hash')
    fields['safety']={**fields['safety'],'alignment_speed_mps':parent['safety']['alignment_speed_mps']}
    original={k:v for k,v in parent.items() if k!='config_hash'}
    xml=ElementTree.parse(out/'scene/world.sdf').getroot()
    physical=json.loads(xml.find("world/model[@name='vehicle']/plugin[@name='r02::WheelDrive']/config").text)
    nav=yaml.safe_load((out/'scene/connector_nav.yaml').read_text())['controller_server']['ros__parameters']
    plan=cfg['connector_qualification']
    with (out/'mission_events.jsonl').open() as f:
        events=list(map(json.loads,f))
    source=out/'measured_front_steering.jsonl'
    measured=list(map(json.loads,source.open())) if source.exists() else []
    transfers=[]
    for sid in plan['segments'][1:]:
        arms=[e for e in events if e.get('segment')==sid and e['event']=='DIRECTION_ARMED_WHILE_STOPPED']
        row=dict(segment=sid,arming_sim_s=arms[0]['t'] if arms else None,alignment_samples=[],sample_columns=['t','left_rad','right_rad','request_v','limited_v'])
        if arms:
            rows=[d for d in measured if d.get('segment')==sid and arms[0]['t']<=d['t']<=arms[0]['t']+3. and
                  d.get('request_t') is not None and abs(d['t']-d['request_t'])<.12 and isinstance(d.get('request_v'),(int,float)) and abs(d['request_v'])>.035]
            first_motion=next((d['t'] for d in rows if abs(d['limited_v'])>=.015),None)
            samples=[d for d in rows if abs(d['limited_v'])<=1.e-5 and (first_motion is None or d['t']<first_motion)]
            row.update(first_measured_motion_s=first_motion,alignment_samples=[[d['t'],*d['front_rad'],d['request_v'],d['limited_v']] for d in samples],
                       sample_count=len(samples),maximum_front_angle_change_rad=max((abs(a-b) for a,b in zip(samples[0]['front_rad'],samples[-1]['front_rad'])),default=0.) if samples else None)
        transfers.append(row)
    files=[out/'scene/connector_nav.yaml',out/'scene/vehicle.json',out/'scene/assembly.json',out/'bt.json',out/'scene/world.sdf']
    checks=dict(same_geometry_and_physical_gains=fields==original,zero_alignment_speed=car['safety']['alignment_speed_mps']==0.,
                single_effective_vehicle_hash=car==physical==cfg['vehicle'] and car['config_hash']==canonical({k:v for k,v in car.items() if k!='config_hash'})
                     and read(out/'bt.json')['vehicle_hash']==car['config_hash'] and car['alignment_parent_hash']==parent['config_hash'],
                versioned_plugin=nav['TurnRPP']['plugin']=='r10::DirectedPathCurvature',unchanged_curvature=nav['TurnRPP']['vehicle_max_curvature']==car['steering']['max_curvature'],
                original_collision_projection_enabled=nav['TurnRPP']['use_collision_detection'] and nav['TurnRPP']['max_allowed_time_to_collision_up_to_carrot']==2.,
                tighter_internal_target=nav['goal']['xy_goal_tolerance']==.025 and nav['goal']['yaw_goal_tolerance']==.04)
    return dict(scope='SDF-to-mission identity and independent measurement of steer-before-roll at the first accepted goal after each change of gear.',
                session=out.name,checks=checks,configuration_passed=all(checks.values()),original_vehicle_hash=parent['config_hash'],vehicle_hash=car['config_hash'],
                original_acceptance=[.10,.12],internal_control_target=[.025,.04],transfers=transfers,
                measured_encoder_file_sha256=sha(source) if source.exists() else None,
                loaded_config_hashes={str(f.relative_to(out)):sha(f) for f in files})


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('session',type=Path)
    ap.add_argument('output',type=Path)
    args=ap.parse_args()
    report=audit(args.session)
    dump(args.output,report)
    print(json.dumps({'configuration_passed':report['configuration_passed'],'rows':[{'segment':t['segment'],'stopped_steer_samples':t.get('sample_count')} for t in report['transfers']]}))
    raise SystemExit(0 if report['configuration_passed'] else 2)
