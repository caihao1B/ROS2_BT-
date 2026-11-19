"""Isolated Ackermann qualification using the existing stationary yaw calibration and recovery stop worker."""
import argparse
import json
import math
from pathlib import Path
import hashlib
import sys
from xml.etree import ElementTree

from common import APP, PKG, ROOT, dump, edition, read, sha
from geometry_r02 import write
from overlay_r08 import overlay
from scene_mowing_r10 import compose
from recovery_stop_candidate_r10 import RecoveryStopEvidence, source_for_candidate


SCRIPTS = PKG / 'scripts'
PROFILES = {
    'late-warm': dict(start=21, count=7, seconds=660),
    'entry': dict(start=0, count=5, seconds=480),
    'full': dict(start=0, count=0, seconds=1800),
}
CANDIDATE_SOURCES = (
    'mowing_yaw_candidate_r10.py', 'spray_inputs_r10.py', 'stationary_gyro_r10.py',
    'recovery_stop_candidate_r10.py', 'runtime_mowing_r10.py', 'mission_mowing_r10.py',
)
LATE_CASE = 'late-warm'


def replace_once(source, marker, replacement):
    if source.count(marker) != 1:
        raise ValueError(f'Frozen implementation extension site changed: {marker}')
    return source.replace(marker, replacement)


def candidate_scene(out, task, session, rtf, cluster, case, segment_start):
    config = compose(out, task, session, rtf, cluster, case='nominal', segment_start=0)
    if not segment_start:
        return config
    if cluster != 'B1' or segment_start != PROFILES[LATE_CASE]['start'] or case != 'nominal':
        raise ValueError('Unknown late-segment fixture')
    mission = read(edition('R10') / 'mowing/mission.json')
    route = next(c for c in mission['clusters'] if c['cluster'] == cluster)
    pose = route['segments'][segment_start]['poses_world'][0]
    tree = ElementTree.parse(out / 'world.sdf').getroot()
    node = tree.find("world/model[@name='vehicle']/pose")
    values = list(map(float, node.text.split()))
    values[0], values[1], values[5] = pose
    node.text = ' '.join(map(str, values))
    write(out / 'world.sdf', tree)
    fixture = read(out / 'fixture.json')
    fixture.update(candidate_profile=LATE_CASE, spawn=pose, coarse_operator_map_hint=[pose[0]-28+.5, pose[1]-14-.5, pose[2]+.05],
                   scope='Diagnostic starts at original approved route entry for segment 21; one usual coarse hint. No live pose resets.')
    dump(out / 'fixture.json', fixture)
    return config


def candidate_overlay(task, out):
    adapt = overlay(task, out)
    if read(out.parent / 'scene/fixture.json').get('candidate_profile') == LATE_CASE:
        # Inject only in generation before sensor delivery. The online estimator
        # never receives the schedule, true robot motion, or the generator bias.
        generator = adapt / 'adapter.py'
        marker = "angular[2]+=f.get('gyro_z_bias_rad_s',0)"
        generated = replace_once(generator.read_text(), marker,
                                 marker + ";angular[2]-=.005*min(1.,max(0.,(t-12.)/3.))")
        compile(generated, str(generator), 'exec')
        generator.write_text(generated)
        manifest = read(adapt / 'adaptation.json')
        manifest['generated']['adapter.py'] = sha(generator)
        manifest['test_sensor_profile'] = dict(after_startup=True, start_s=12., ramp_s=3.,
                                               added_yaw_bias_rad_s=-.005, scope='SIMULATED_SENSOR_ONLY')
        dump(adapt / 'adaptation.json', manifest)
    return adapt


def worker_source():
    source = source_for_candidate((SCRIPTS / 'mission_mowing_r10.py').read_text())
    marker = "  log('NAV_LIFECYCLE_ACTIVE_VERIFIED')"
    extra = """
  if spec.get('candidate_profile')=='late-warm':
   begin=st['t']
   log('WARM_TEST_STOP_BEGIN')
   wait(lambda:st['t']>=max(45.,begin+15.),45)
   log('WARM_TEST_STOP_END',began=begin)
"""
    return replace_once(source, marker, marker + extra)


def runtime(profile):
    path = SCRIPTS / 'runtime_mowing_r10.py'
    text = path.read_text()
    text = replace_once(text, 'from scene_mowing_r10 import compose', '')
    text = replace_once(text, 'from overlay_r08 import overlay', '')
    text = replace_once(text, "APP/'results/R10'/(task+'_'+stamp())", "APP/'results/R10/candidates'/(task+'_'+stamp())")
    text = replace_once(text, "dump(APP/'results/R10'/(task+'_'+case+'_latest.json')",
                        "dump(APP/'results/R10/candidates'/(candidate_profile+'_latest.json')")
    text = replace_once(text, "spec['online_probe_sha256']=sha(PKG/'scripts/mission_mowing_r10.py')",
                        "spec.update(candidate_profile=candidate_profile,formal_credit=False,online_probe_sha256=sha(Path(__file__)))")
    text = replace_once(text, "spec['implementation_bindings']=",
                        "bound += [PKG/'scripts'/name for name in candidate_sources];spec['implementation_bindings']=")
    text = replace_once(text, "str(adapt/script if role in ['icp_gate','inputs'] else s05/script)",
                        "str(PKG/'scripts/spray_inputs_r10.py' if role=='inputs' else adapt/script if role=='icp_gate' else s05/script)")
    text = replace_once(text, "str(PKG/'scripts/mission_mowing_r10.py'),'--out'",
                        "str(Path(__file__)),'worker','--out'")
    marker = "bind_adapter(adapt,cfg);spec.update("
    text = replace_once(text, marker, "bind_adapter(adapt,cfg);spec.update(candidate_generated_sha256="
                        "{str(path.relative_to(out)):sha(path) for path in [*adapt.glob('*.py'),adapt/'adaptation.json',"
                        "out/'scene/world.sdf',out/'scene/fixture.json',out/'scene/sensor_manifest.json']},"
                        "candidate_worker_sha256=worker_hash);spec.update(")
    scope = dict(__file__=__file__, __name__='candidate_runtime', candidate_profile=profile,
                 candidate_sources=CANDIDATE_SOURCES, compose=candidate_scene, overlay=candidate_overlay,
                 worker_hash=hashlib.sha256(worker_source().encode()).hexdigest())
    exec(compile(text, str(path), 'exec'), scope)
    return scope['run']


def verify(run):
    from analyze_mowing_r10 import analyze
    from analyze_r02 import rows
    import statistics
    spec = read(run / 'spec.json')
    profile = spec['candidate_profile']
    try:
        result = analyze(run)
    except (FileNotFoundError, ValueError, IndexError, KeyError) as error:
        # Preserve an interrupted/failed startup even without a complete sensor archive.
        report = dict(passed=False, profile=profile, formal_credit=False,
                      failure='BASELINE_ANALYSIS_INCOMPLETE', detail=str(error),
                      runtime=read(run/'runtime.json') if (run/'runtime.json').is_file() else None)
        dump(run/'candidate_analysis.json',report)
        return report
    events = rows(run / 'mission_events.jsonl') if (run / 'mission_events.jsonl').is_file() else []
    observed = [e for e in events if e['event']=='RECOVERY_STOP_FEEDBACK_CONFIRMED']
    pose = read(run / 'mission_result.json') if (run / 'mission_result.json').is_file() else {}
    adapter = read(run / 'inputs/result.json') if (run / 'inputs/result.json').is_file() else {}
    physical = rows(run / 'tool_evaluation/physical.jsonl')
    checks = dict(unchanged_physical_acceptance=bool(result['passed']),
                  bound_generated_sources=bool(spec.get('candidate_generated_sha256')) and all(
                      (run/p).is_file() and sha(run/p)==h for p,h in spec.get('candidate_generated_sha256',{}).items()),
                  bound_executed_worker=hashlib.sha256(worker_source().encode()).hexdigest()==spec.get('candidate_worker_sha256'),
                  stationary_adaptation_exercised=adapter.get('stationary_updates',0)>100,
                  recovery_stop_feedback_observed=bool(observed),
                  recovery_after_new_samples=all(e['t']-e['requested_at']>=e['minimum_span_s'] for e in observed))
    warm = {}
    if profile==LATE_CASE:
        begins = [e['t'] for e in events if e['event']=='WARM_TEST_STOP_BEGIN']
        ends = [e['t'] for e in events if e['event']=='WARM_TEST_STOP_END']
        gates = rows(run / 'control/gate.jsonl')
        samples = rows(run / 'inputs/inputs.jsonl')
        start, end = (begins[0], ends[0]) if len(begins)==len(ends)==1 else (0.,0.)
        hold = [g for g in gates if start<=g['t']<=end]
        rotors = [p for p in physical if start<=p['t']<=end]
        rates = [r['gyro'][2] for r in samples if r['key']=='imu' and end-5.<=r['t']<=end]
        wheels = [r for r in samples if r['key']=='wheel' and end-5.<=r['t']<=end]
        yaw_disagreement = statistics.mean(rates)-statistics.mean(r['w'] for r in wheels) if rates and wheels else None
        max_speed=max((abs(r['body_v']) for r in rotors),default=None)
        max_displacement=max((math.dist(r['body_xyz'][:2],rotors[0]['body_xyz'][:2]) for r in rotors),default=None)
        stop_speed=read(run/'mission.json')['vehicle']['safety']['stopped_v_mps']
        warm = dict(start_s=start,end_s=end,gate_samples=len(hold),physical_tool_samples=len(rotors),
                    maximum_independent_body_speed_mps=max_speed,maximum_independent_body_displacement_m=max_displacement,
                    original_stopped_speed_gate_mps=stop_speed,corrected_gyro_minus_encoder_rad_s=yaw_disagreement)
        checks.update(warm_physical_hold=end-start>=15 and bool(hold) and all(g['mode']=='STOP' and g['stopped'] for g in hold)
                      and bool(rotors) and all(abs(p['rpm_rad_s'])<.1 for p in rotors) and max_speed<stop_speed,
                      held_gyro_consistent_with_encoder=yaw_disagreement is not None and abs(yaw_disagreement)<.0015,
                      target_turn_passed='B1:safe_turn:6:3' in [r['segment'] for r in pose.get('original_segment_passages',[])])
    report=dict(passed=all(checks.values()),checks=checks,profile=profile,
                original_checks=result['checks'],selected_original_segments=len(pose.get('original_segment_passages',[])),
                stop_handshakes=len(observed),warm_hold=warm,adapter=adapter,formal_credit=False,
                scope='Separate candidate profile. A partial-route run qualifies only its explicitly selected measured segments.')
    dump(run / 'candidate_analysis.json', report)
    return report


if __name__=='__main__':
    if sys.argv[1:2]==['worker']:
        del sys.argv[1]
        source=worker_source()
        exec(compile(source, str(SCRIPTS/'mission_mowing_r10.py'), 'exec'),
             dict(__name__='__main__', __file__=str(SCRIPTS/'mission_mowing_r10.py'), RecoveryStopEvidence=RecoveryStopEvidence))
    else:
        parser=argparse.ArgumentParser(description=__doc__)
        parser.add_argument('profile',choices=PROFILES)
        parser.add_argument('--rtf',type=float,default=.4)
        parser.add_argument('--prepare',action='store_true',help='Validate driver and generated fixture in a private temp directory, without starting Gazebo.')
        parser.add_argument('--verify',type=Path,help='Read an existing candidate session and regenerate its separate assessment.')
        args=parser.parse_args()
        settings=PROFILES[args.profile]
        run=runtime(args.profile)
        if args.prepare:
            import tempfile
            with tempfile.TemporaryDirectory(prefix='mow_candidate_') as temp:
                out=Path(temp)
                candidate_scene(out/'scene','mowing','prepare',args.rtf,'B1','nominal',settings['start'])
                candidate_overlay('mowing',out/'overlay')
                compile(worker_source(),str(SCRIPTS/'mission_mowing_r10.py'),'exec')
                print(json.dumps(dict(profile=args.profile,selected=settings,fixture=read(out/'scene/fixture.json'),
                                      generator=sha(out/'overlay/adapter.py'))))
        else:
            session=args.verify or run(task='mowing',seconds=settings['seconds'],rtf=args.rtf,cluster='B1',
                                       segment_limit=settings['count'],case='nominal',segment_start=settings['start'])
            report=verify(session)
            print(json.dumps(report,indent=2))
            raise SystemExit(0 if report['passed'] else 2)
