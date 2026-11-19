"""Prepare or drive the frozen seed877 46-section mission from the original B1 entrance."""
import argparse
import gzip
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

from common import APP, PKG, ROOT, dump, read, sha
from geometry_r02 import write
from mowing_yaw_candidate_r10 import replace_once
import qualify_b1_context as context

steering = context.steering
baseline = steering.baseline
FROZEN = ROOT / 'validation/r10_continuation_20261119/b1'
RESULTS = APP / 'results/experiments/b1_full_46'


def frozen_mission():
    receipt = read(FROZEN / 'full_plan_preflight.json')
    archive = FROZEN / 'mission_46.json.gz'
    if sha(archive) != receipt['compressed_full_edition']['sha256']:
        raise ValueError('Frozen full B1 edition archive changed')
    frozen = json.loads(gzip.decompress(archive.read_bytes()))
    cfg, _, _ = context.mission(paired=True)
    if cfg != frozen or cfg['version'] != receipt['candidate_version']:
        raise ValueError('Built mission differs from the physically tested seed877 edition')
    cluster = next(c for c in cfg['clusters'] if c['cluster'] == 'B1')
    ids = [s['id'] for s in cluster['segments']]
    if ids != receipt['full_ordered_sections'] or len(set(ids)) != 46:
        raise ValueError('46 unique ordered segments are required')
    return cfg, cluster


def entrance_scene(out, task, session, **kwargs):
    if task != 'mowing' or kwargs['cluster'] != 'B1' or kwargs['segment_start'] != 0:
        raise ValueError('Only the full B1 entrance trial is supported')
    cfg, cluster = frozen_mission()
    # Reuse the physically qualified drivetrain, map, and Nav2 plugin. Replace its
    # isolated fixture with the original entrance before any process sees the scene.
    connector_index = cfg['connector_qualification']['start_index']
    settings = dict(kwargs, segment_start=connector_index)
    scene = steering.steering_scene(out, task, session, **settings)
    if read(out/'connector_mission.json') != cfg:
        raise ValueError('Scene plan and frozen edition disagree')
    first = cluster['segments'][0]
    tree = ElementTree.parse(out/'world.sdf').getroot()
    pose = tree.find("world/model[@name='vehicle']/pose")
    values = list(map(float, pose.text.split()))
    values[0], values[1], values[5] = first['poses_world'][0]
    pose.text = ' '.join(map(str, values))
    write(out/'world.sdf', tree)
    x, y, yaw = first['poses'][0]
    dump(out/'fixture.json', dict(candidate_profile=context.PROFILE, spawn=first['poses_world'][0],
            online_access=False, coarse_operator_map_hint=[x+.5, y-.5, yaw+.05],
            safe_plan_revision=cfg['safe_execution_plan']['revision'], plan_revision=cfg['version'],
            start_index=0, selection='ALL_46', scope='Whole cluster from the original entrance; one biased operator hint.'))
    return scene


def configure(wall_seconds):
    steering.PROFILE = baseline.PROFILE = context.PROFILE
    baseline.worker_source = context.worker
    baseline.mission_version = context.mission
    baseline.candidate_scene = entrance_scene
    baseline.RESULTS = RESULTS
    binary = steering.build_controller()
    runtime = context.driver(binary).read_text()
    runtime = replace_once(runtime, 'wall_budget=max(600,int(seconds/min(rtf,.25))+90)',
                           'wall_budget='+str(wall_seconds))
    runtime = replace_once(runtime, "bound += [PKG/'scripts/bridge_r10.py'",
        "bound += [PKG/'scripts/qualify_b1_full.py', PKG/'scripts/qualify_b1_context.py', "
        "PKG/'scripts/qualify_b1_steering.py', ROOT/'validation/r10_continuation_20261119/b1/mission_46.json.gz'];bound += [PKG/'scripts/bridge_r10.py'")
    target = steering.BUILD / ('full_entrance_runtime_'+str(wall_seconds)+'.py')
    if not target.is_file() or target.read_text() != runtime:
        target.write_text(runtime)
    baseline.DRIVER = target
    baseline.__file__ = __file__
    return baseline.runtime(paired=True)


def main():
    if sys.argv[1:2] == ['worker']:
        del sys.argv[1]
        exec(compile(context.worker(), str(baseline.WORKER), 'exec'),
             dict(__name__='__main__', __file__=str(baseline.WORKER),
                  RecoveryStopEvidence=steering.RecoveryStopEvidence, PROFILE=context.PROFILE,
                  RetryProgress=steering.RetryProgress, disposition=context.disposition))
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('prepare','run'))
    parser.add_argument('--rtf',type=float,default=.85)
    parser.add_argument('--wall-seconds',type=int,default=3000,help='Whole-run watchdog; safe shutdown follows.')
    args = parser.parse_args()
    if not (0 < args.rtf <= 1 and 60 <= args.wall_seconds <= 7200):
        parser.error('rtf must be in (0,1] and wall-seconds in [60,7200]')
    run = configure(args.wall_seconds)
    cfg, cluster = frozen_mission()
    opts = dict(task='mowing',case='nominal',cluster='B1',segment_start=0,rtf=args.rtf)
    if args.mode=='prepare':
        with TemporaryDirectory() as temp:
            out=Path(temp)/'scene'
            entrance_scene(out,session='prepare-full',paired=True,**opts)
            compile(context.worker(),str(baseline.WORKER),'exec')
            print(json.dumps(dict(fixture=read(out/'fixture.json'),count=len(cluster['segments']),
                       version=cfg['version'],limits=cfg['navigation_execution']['stopped_arrival_confirmation'],
                       budget=cluster['budget_s'],wall_budget=args.wall_seconds)))
        return
    directory=run(**opts,segment_limit=0,seconds=cluster['budget_s'])
    from score_b1_full import evaluate
    report=evaluate(directory)
    print(json.dumps({k:report[k] for k in ['status','session','completed_segments','actual_cut_m2','residual_m2']}))
    raise SystemExit(0 if report['status']=='PASS' else 2)


if __name__ == '__main__':
    main()
