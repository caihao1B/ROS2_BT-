"""Run only the four seed877 turn sections on the actual R12 Gazebo vehicle. No coverage credit."""
import argparse
from functools import partial
import hashlib
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

from connector_registration import overlay as trimmed_overlay
from common import APP, PKG, ROOT, canonical, dump, edition, read, sha
from geometry_r02 import write
from mowing_yaw_candidate_r10 import replace_once
from recovery_stop_candidate_r10 import RecoveryStopEvidence, source_for_candidate
from scene_mowing_r10 import compose as original_scene

sys.path.insert(0, str(APP))
from late_connector import PREFIX, ROUTE, arc_length, compare
from recovery_mowing_r10 import gear_runs

PROFILE = 'B1-seed877-four-gears'
RESULTS = APP / 'results/experiments/b1_connector'
WORKER = PKG / 'scripts/mission_mowing_r10.py'
DRIVER = edition('R12') / 'runtime_mowing_video.py'


def replacement_segments(proposal, offset, template):
    """Build the exact same directed segments for offline geometry and physical execution."""
    replacements = []
    for i, gear in enumerate(gear_runs(proposal)):
        poses = gear['poses']
        length = arc_length(poses)
        segment = dict(template, id=f'B1:offline_connection:6:{i}', direction=gear['direction'],
                       poses=poses, poses_world=[[x+offset[0],y+offset[1],h] for x,y,h in poses],
                       length_m=length, nominal_seconds=length/.35)
        replacements.append(segment)
    return replacements


def mission_version(path=ROUTE, paired=False):
    geometry = compare(path)
    if not geometry['passed']:
        raise ValueError('Candidate is outside the original geometric authority')
    cfg = read(edition('R10') / 'mowing/mission.json')
    cluster = next(c for c in cfg['clusters'] if c['cluster'] == 'B1')
    original = cluster['segments']
    indices = [i for i, s in enumerate(original) if s['id'].startswith(PREFIX)]
    if [original[i]['id'] for i in indices] != geometry['covered_original_connector_ids']:
        raise ValueError('Original path differs from the checked connection')
    candidate = read(path)
    sections = replacement_segments(candidate['points'], cfg['map_offset'], original[indices[0]])
    if len(sections) != 4 or any(s['tool_on'] for s in sections):
        raise ValueError('Expected four tool-off sections')
    cluster['segments'][indices[0]:indices[-1]+1] = sections
    cluster['motion_m'] = sum(s['length_m'] for s in cluster['segments'])
    cluster['tool_off_m'] = cluster['motion_m'] - cluster['work_m']
    cluster['estimated_seconds'] = geometry['proposed_full_plan_estimate_s']
    cfg['navigation_execution']['current_execution_segments'] = len(cluster['segments'])
    cfg['connector_qualification'] = dict(parent_mission_version=cfg.pop('version'), geometry_revision=candidate['revision'],
                                          geometry_sha256=sha(path), registration='train-only trim 60%, Huber 4 cm; unmodified held-out and independent RTK gates',
                                          gear_stop='fresh tool/vehicle observations plus original minimum 15 samples / 0.6 s; offline physical pose cross-check',
                                          start_index=indices[0], segments=[s['id'] for s in sections],
                                          selected_scope='Four physical tool-off sections with stopped admissions; zero work credit.')
    if paired:
        cfg['connector_qualification']['terrain_pairing']='R09 robot self-filter with capture-time odometry pairing, unchanged 30 ms limit; bounded deferred scans'
    cfg['version'] = canonical(cfg)
    return cfg, sections, geometry


def candidate_scene(out, task, session, paired=False, **kwargs):
    cfg, sections, geometry = mission_version(paired=paired)
    index = cfg['connector_qualification']['start_index']
    if kwargs.get('segment_start') != index or task != 'mowing' or kwargs.get('cluster') != 'B1':
        raise ValueError('Wrong physical qualification requested')
    config = original_scene(out, task, session, **{**kwargs, 'segment_start': 0})
    dump(out/'connector_mission.json', cfg)
    dump(out/'connector_geometry.json', geometry)
    manager = read(edition('R10')/'mowing/manager.json')
    manager['plan_revision'] = cfg['version']
    dump(out/'connector_manager.json', manager)
    tool = read(edition('R10')/'mowing/tool.json')
    tool.pop('tool_revision')
    tool['plan_revision'] = cfg['version']
    tool['tool_revision'] = canonical(tool)
    tool['session'] = session
    dump(out/'tool.json', tool)
    tree = ElementTree.parse(out/'world.sdf').getroot()
    vehicle = tree.find("world/model[@name='vehicle']")
    vehicle.find("plugin[@name='s10::ToolActuator']/config").text = json.dumps(tool)
    spawn = sections[0]['poses_world'][0]
    pose = vehicle.find('pose')
    values = list(map(float, pose.text.split()))
    values[0], values[1], values[5] = spawn
    pose.text = ' '.join(map(str, values))
    write(out/'world.sdf', tree)
    p = sections[0]['poses'][0]
    dump(out/'fixture.json', dict(candidate_profile=PROFILE, spawn=spawn, online_access=False,
         coarse_operator_map_hint=[p[0]+.5, p[1]-.5, p[2]+.05], safe_plan_revision=cfg['safe_execution_plan']['revision'],
         plan_revision=cfg['version'], scope='Independent stopped start at the candidate connector entry. Single coarse hint; online pose is entirely sensor-derived.'))
    return config


def worker_source():
    source = source_for_candidate(WORKER.read_text())
    # A gate state leftover from driving is insufficient. Require fresh physical encoder and
    # blade messages for each stop including the very first stopped gear authorization.
    source = replace_once(source, "  if st.get('endpoint_recovery'):", "  if st.get('endpoint_recovery') or spec.get('candidate_profile')==PROFILE:")
    marker = "if dist>.10 or angle>.12:raise RuntimeError('SEGMENT_ARRIVAL_NOT_CONFIRMED')"
    if source.count(marker) != 2:
        raise ValueError('Both direct and recovered terminal branches must enforce the stopped sample window')
    source = source.replace(marker, "if dist>.10 or angle>.12 or not stopped_original_arrival(s):raise RuntimeError('SEGMENT_ARRIVAL_NOT_CONFIRMED')")
    return source


def runtime(paired=False):
    text = DRIVER.read_text()
    terrain_bindings=[]
    if paired:
        from terrain_pairing_mowing_r10 import bindings, build
        paired_binary=build()
        terrain_bindings=[*bindings(),PKG/'scripts/terrain_pairing_mowing_r10.py',edition('R10')/'mowing/terrain.json']
        text=replace_once(text, "launch('terrain',[str(APP/'build/r09/r09_terrain'),", "launch('terrain',["+repr(str(paired_binary))+",")
    text = replace_once(text, 'from scene_mowing_r10 import compose as original_compose', '')
    text = replace_once(text, 'from overlay_r08 import overlay', '')
    text = replace_once(text, "APP/'results/R12'/(task+'_'+stamp())", "candidate_results/(task+'_'+stamp())")
    text = replace_once(text, "APP/'results/R12'/(task+'_'+case+'_latest.json')", "candidate_results/'latest.json'")
    text = replace_once(text, "cfg=read((edition('R10')/'mowing')/'mission.json')", "cfg=read(out/'scene/connector_mission.json')")
    text = replace_once(text, "str((edition('R10')/'mowing')/'manager.json')", "str(out/'scene/connector_manager.json')")
    text = replace_once(text, "str(adapt/script if role in ['icp_gate','inputs'] else s05/script)",
        "str(PKG/'scripts/spray_inputs_r10.py' if role=='inputs' else adapt/script if role=='icp_gate' else s05/script)")
    text = replace_once(text, "str(PKG/'scripts/mission_mowing_r10.py'),'--out'", "str(Path(__file__)),'worker','--out'")
    text = replace_once(text, "spec['online_probe_sha256']=sha(PKG/'scripts/mission_mowing_r10.py')",
        "spec.update(candidate_profile=profile,formal_credit=False,online_probe_sha256=sha(Path(__file__)),candidate_worker_sha256=worker_hash)")
    text = replace_once(text, "spec['implementation_bindings']=", "bound += named_bindings;spec['implementation_bindings']=")
    text = replace_once(text, "bind_adapter(adapt,cfg);spec.update(", "bind_adapter(adapt,cfg);spec.update(generated_bindings="
        "{str(p.relative_to(out)):sha(p) for p in [out/'scene/world.sdf',out/'scene/connector_manager.json',out/'scene/connector_mission.json',"
        "out/'scene/tool.json',out/'scene/fixture.json',out/'overlay/adaptation.json',out/'overlay/regional_math.py']},")
    bound = [Path(__file__), Path(__file__).with_name('score_b1_connector.py'), DRIVER, ROUTE, APP/'late_connector.py',
             PKG/'scripts/spray_inputs_r10.py',PKG/'scripts/stationary_gyro_r10.py',PKG/'scripts/mowing_yaw_candidate_r10.py',
             PKG/'scripts/recovery_stop_candidate_r10.py',PKG/'scripts/connector_registration.py',
             PKG/'scripts/visual_r11.py',PKG/'scripts/transport_video_r12.py',PKG/'scripts/record_r12.py']
    scope = dict(__file__=__file__, __name__='connector_runtime', original_compose=partial(candidate_scene,paired=paired), overlay=trimmed_overlay, profile=PROFILE,
                 candidate_results=RESULTS, named_bindings=bound+terrain_bindings, worker_hash=hashlib.sha256(worker_source().encode()).hexdigest())
    exec(compile(text, str(DRIVER), 'exec'),scope)
    return scope['run']


if __name__ == '__main__':
    if sys.argv[1:2] == ['worker']:
        del sys.argv[1]
        exec(compile(worker_source(), str(WORKER), 'exec'), dict(__name__='__main__', __file__=str(WORKER),
             RecoveryStopEvidence=RecoveryStopEvidence, PROFILE=PROFILE))
    else:
        parser=argparse.ArgumentParser(description=__doc__)
        parser.add_argument('command', choices=['prepare','run'])
        parser.add_argument('--rtf', type=float, default=.55)
        parser.add_argument('--terrain',choices=['capture-time','arrival-order'],default='capture-time',
            help='Use R09 terrain paired to the LiDAR capture timestamp. Arrival order reproduces the first legacy experiment.')
        parser.add_argument('--seconds', type=float, default=390)
        args=parser.parse_args()
        paired=args.terrain=='capture-time'
        run = runtime(paired)
        cfg, sections, _ = mission_version(paired=paired)
        settings=dict(task='mowing',cluster='B1',case='nominal',rtf=args.rtf,segment_start=cfg['connector_qualification']['start_index'])
        if args.command == 'prepare':
            with TemporaryDirectory() as temp:
                out=Path(temp)/'scene'
                candidate_scene(out, session='prepare', paired=paired, **settings)
                trimmed_overlay('mowing',Path(temp)/'overlay')
                compile(worker_source(),str(WORKER),'exec')
                print(json.dumps(dict(scope=PROFILE, fixture=read(out/'fixture.json'), sections=[dict(id=s['id'], direction=s['direction'],length_m=s['length_m'],goal=s['poses'][-1]) for s in sections]),indent=2))
        else:
            session=run(**settings,segment_limit=len(sections),seconds=args.seconds)
            from score_b1_connector import evaluate
            report=evaluate(session)
            print(json.dumps(report,indent=2,allow_nan=False))
            raise SystemExit(0 if report['passed'] else 2)
