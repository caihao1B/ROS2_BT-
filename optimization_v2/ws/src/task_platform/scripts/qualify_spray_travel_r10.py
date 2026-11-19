"""Check or run the reduced-turn A1 route with the established precision worker and physical scorer."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from xml.etree import ElementTree

from common import APP, PKG, ROOT, canonical, dump, edition, read, sha

SCRIPTS = PKG / 'scripts'
MISSION = edition('R10') / 'spray/mission.json'
PLAN = APP / 'candidates/A1_equal_length_turns.json'


def travel_points(allpoints, selected, cfg):
    """Only an exact prefix of the bound 56-point plan can be dispatched."""
    session = cfg['precision_session']
    ids = session['ordered_ids']
    records = {p['id']: p for p in allpoints}
    if (len(records) != len(ids) or len(ids) != 56 or set(records) != set(ids) or
            ids[0] != allpoints[0]['id'] or ids[-1] != allpoints[-1]['id'] or
            len(selected) < 2 or selected != ids[:len(selected)] or
            selected != session['selected_ids']):
        raise ValueError('Invalid obligation set, endpoint or candidate prefix')
    if cfg['navigation_goal_orientation_required'] is not False:
        raise ValueError('The reduced-turn plan requires position-only service goals')
    result = []
    for i, pid in enumerate(selected):
        row = records[pid].copy()
        if i:
            x, y = records[ids[i-1]]['target']
            dx, dy = row['target'][0] - x, row['target'][1] - y
            row['yaw'] = math.atan2(dy, dx)
        result.append(row)
    return result


def verify_plan(certify=False):
    if str(APP) not in sys.path:
        sys.path.insert(0, str(APP))
    import numpy as np
    from spray_equal_length import angle, build_plan
    from spray_route_time import model
    mission, plan = read(MISSION), read(PLAN)
    if plan['source_mission_sha256'] != sha(MISSION) or plan['source_mission_version'] != mission['version']:
        raise ValueError('The frozen mission has changed')
    cluster = next(c for c in mission['clusters'] if c['id'] == 'A1')
    ids = plan['ordered_ids']
    points, _, _, report, _, _ = model(mission, cluster)
    index = {r['id']: i for i, r in enumerate(cluster['points'])}
    order = [index[pid] for pid in ids]
    if sorted(order) != list(range(len(points))) or order[0] != 0 or order[-1] != len(points)-1:
        raise ValueError('Candidate changed an obligation or endpoint')
    original, candidate = report(np.arange(len(points))), report(np.array(order))
    ingress = [cluster['start'][2], *(angle(points[a], points[b]) for a, b in zip(order, order[1:]))]
    if (original != plan['original'] or candidate != plan['candidate'] or
            dict(zip(ids, ingress)) != plan['ingress_yaw_rad'] or
            candidate['open_motion_m'] > original['open_motion_m'] or
            candidate['sum_abs_heading_change_rad'] >= original['sum_abs_heading_change_rad']):
        raise ValueError('The published certificate and current route geometry disagree')
    if certify:
        # Different HiGHS versions may select another equally good path. Check
        # the published feasible solution against the independently solved bound.
        fresh = build_plan(mission)
        if not (fresh['certificate']['turn_lower_bound_rad'] == candidate['sum_abs_heading_change_rad']
                == plan['certificate']['turn_lower_bound_rad']):
            raise ValueError('The committed path does not achieve the certified bound')
    return mission, plan


def prepare(count):
    from spray_precision_session_r10 import terrain_version, worker
    from spray_defer_fixture_r10 import once
    from terrain_pairing_r10 import CMAKE, INCLUDE, SOURCE

    base, plan = verify_plan()
    if not 2 <= count <= len(plan['ordered_ids']):
        raise ValueError('Select a prefix of 2–56 candidate points from the original depot')
    ids = plan['ordered_ids'][:count]
    fixture_file = SCRIPTS / 'spray_defer_fixture_r10.py'
    spec = importlib.util.spec_from_file_location('travel_fixture', fixture_file)
    fixture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    parent = worker()
    text = once(parent.read_text(), 'from spray_defer_policy_r10 import selected_points,',
                'from qualify_spray_travel_r10 import travel_points\nfrom spray_defer_policy_r10 import ')
    text = once(text, "points=selected_points(allpoints,spec['candidate_selected_point_ids'])",
                "points=travel_points(allpoints,spec['candidate_selected_point_ids'],cfg)")
    target = APP / 'build/spray_travel/mission_spray_travel.py'
    target.parent.mkdir(exist_ok=True, parents=True)
    compile(text, str(target), 'exec')
    target.write_text(text)
    binary, terrain = terrain_version()
    sources = [*fixture.FIXTURE_SOURCES, parent, target, fixture.RECORDER, PLAN, MISSION, APP/'spray_equal_length.py',
               APP/'spray_route_time.py', INCLUDE, SOURCE, CMAKE, terrain.INCLUDE, terrain.OUT/'terrain.cpp', binary,
               *(SCRIPTS/k for k in ('spray_handoff_candidate_r10.py', 'mission_spray_r10.py', 'spray_precision_r10.py',
                 'spray_precision_candidate_r10.py', 'spray_precision_session_r10.py', 'terrain_pairing_r10.py')),
               Path(__file__).resolve()]
    bound = {str(p.relative_to(ROOT)): sha(p) for p in sources}
    cfg = dict(base)
    cfg['precision_session'] = dict(parent_version=cfg.pop('version'), ordered_ids=plan['ordered_ids'],
                                    selected_ids=ids, original_obligations=len(plan['ordered_ids']),
                                    route_file_sha256=sha(PLAN), formal_credit=False, bound_implementation=bound)
    cfg['version'] = canonical(cfg)
    cluster = next(c for c in cfg['clusters'] if c['id'] == 'A1')
    chosen = travel_points(cluster['points'], ids, cfg)
    fixture.FOUR = ids
    fixture.original_points = lambda: (cfg, chosen)

    def scene(out, task, session, **kwargs):
        return fixture.fixture_scene(out, task, session, False, cfg['version'], **kwargs)

    template = once(fixture.runtime_source(), "launch('terrain',[str(APP/'build/r08/r08_terrain')",
                    "launch('terrain',[str(APP/'"+str(binary.relative_to(APP))+"')")
    scope = dict(__file__=str(fixture_file), __name__='a1_travel_runtime', compose=scene, candidate_config=lambda: cfg,
                 EXPERIMENTS=APP/'results/experiments/spray_travel', latest_name='latest.json', selected_ids=ids,
                 worker=target, RECORDER=fixture.RECORDER, FIXTURE_SOURCES=sources)
    exec(compile(template, str(fixture.RECORDER), 'exec'), scope)
    return cfg, scene, scope['run']



def check_scene(cfg, compose):
    """Build the same task scene the recorder would use, and check the offline version transaction."""
    baseline = read(MISSION)
    original = dict(cfg)
    session = original.pop('precision_session')
    original['version'] = session['parent_version']
    if original != baseline or cfg['version'] != canonical({k: v for k, v in cfg.items() if k != 'version'}):
        raise ValueError('The parent mission or candidate version is inconsistent')
    if any(sha(ROOT / path) != expected for path, expected in session['bound_implementation'].items()):
        raise ValueError('A bound input changed after candidate preparation')
    with tempfile.TemporaryDirectory(prefix='a1-travel-check-') as folder:
        out = Path(folder) / 'scene'
        compose(out, 'spray', 'offline_travel', cluster='A1', rtf=.55)
        fixture, manager, tool = (read(out/name) for name in ('fixture.json', 'manager.json', 'tool.json'))
        physical = json.loads(ElementTree.parse(out/'world.sdf').findtext(
            ".//model[@name='vehicle']/plugin[@name='s10::ToolActuator']/config"))
        if (fixture['selected_original_ids'] != session['selected_ids'] or physical != tool or
                any(version != cfg['version'] for version in
                    (fixture['candidate_version'], manager['plan_revision'], tool['plan_revision']))):
            raise ValueError('Recorder, localization and physical tool were bound to different plans')
    return {'fixture_checked_offline': True, 'candidate_version': cfg['version'],
            'selected': session['selected_ids'], 'route_sha256': sha(PLAN), 'formal_credit': False}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('replay', help='Recalculate the observational comparison from published point cycles')
    commands.add_parser('check', help='Verify every leg, work obligation and turn bound without starting Gazebo')
    preview = commands.add_parser('prepare', help='Build source-bound dispatcher and terrain executable without Gazebo')
    preview.add_argument('--count', type=int, default=4)
    live = commands.add_parser('run', help='Drive an independent optional prefix from the depot in Gazebo')
    live.add_argument('--count', type=int, default=4)
    live.add_argument('--seconds', type=int, default=240)
    live.add_argument('--rtf', type=float, default=.55)
    args = parser.parse_args()
    if args.command == 'replay':
        if str(APP) not in sys.path:
            sys.path.insert(0, str(APP))
        from spray_travel_replay import build, REPORT
        report = build()
        if report != read(REPORT):
            raise ValueError('The published replay differs from its bound inputs')
        print(json.dumps(report, indent=2))
        return 0
    if args.command == 'check':
        _, plan = verify_plan(certify=True)
        print(json.dumps({k: plan[k] for k in ('schema', 'original', 'candidate', 'certificate', 'scope')}, indent=2))
        return 0
    cfg, compose, runner = prepare(args.count)
    if args.command == 'prepare':
        print(json.dumps(check_scene(cfg, compose), indent=2))
        return 0
    if not 60 <= args.seconds <= cfg['task_budget_s'] or not 0 < args.rtf <= 1:
        raise ValueError('Stay within the original simulation budget and a real-time factor in (0,1]')
    from analyze_spray_precision_r10 import evaluate
    session = Path(runner(task='spray', seconds=args.seconds, rtf=args.rtf, cluster='A1',
                          point_limit=args.count, case='travel_equal_length'))
    report = evaluate(session, 'travel_equal_length')
    report['scope'] = 'Reduced-turn prefix from depot; original 56-point dose denominator retained'
    dump(session/'travel_work_report.json', report)
    print(session, report['completed_count'], report['selected_count'], report['measured_subset_pass'])
    return 0 if report['measured_subset_pass'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
