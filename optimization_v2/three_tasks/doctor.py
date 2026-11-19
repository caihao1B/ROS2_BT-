#!/usr/bin/env python3
"""Read-only diagnostics for a three-task checkout. No simulator or receipt writes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'optimization_v2'
APP = BASE / 'three_tasks'
SCRIPTS = BASE / 'ws/src/task_platform/scripts'
sys.path.insert(0, str(SCRIPTS))
sys.path.append(str(BASE / 'ws/src/task_runtime/scripts'))
TASKS = ('spray', 'inspection', 'mowing')


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def verify_digest(root, relative, expected):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f'Path outside the input directory: {relative}')
    with path.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != expected:
            raise ValueError(f'SHA-256 mismatch: {relative}')


def assets():
    from common_r1 import canonical
    from contract_r05 import verify_runtime_contract

    frozen_root = BASE / 'maps/frozen'
    frozen = read(frozen_root / 'manifest.json')
    plans_root = BASE / 'plans/s17_full'
    plan_files = read(plans_root / 'manifest.json')
    outputs = {}
    for task in TASKS:
        binding_path = APP / 'editions/R05' / task / 'binding.json'
        binding = read(binding_path)
        os.environ['R05_BINDING'] = str(binding_path)
        request = dict(rig_binding_revision=binding['revision'], map_id=frozen['map_id'],
                       rig_revision=binding['rig_revision'])
        # Use the localization node's real admission function, including the immutable DB.
        verify_runtime_contract(request)
        outputs[task] = {'map_id': frozen['map_id'], 'rig_revision': binding['rig_revision']}

    selector = read(BASE / 'evaluation/active_inputs17.json')
    atlas = read(ROOT / selector['atlas'])
    profile = read(APP / 'config/tasks/spray.json')
    for name in ('atlas', 'plan'):
        expected = profile['bindings'][name]
        if selector[name] != expected['path']:
            raise ValueError(f'Active {name} path differs from the bound input')
        verify_digest(ROOT, selector[name], expected['sha256'])
    if plan_files['atlas_revision'] != atlas['revision']:
        raise ValueError('Regional atlas and task plan revisions differ')
    for region in atlas['regions'].values():
        for key in ('cloud_path', 'navigation_yaml', 'navigation_pgm',
                    'occupancy_evidence', 'occupancy_summary'):
            relative = region[key]
            verify_digest(ROOT, relative, atlas['bindings'][relative])
    for relative, expected in plan_files['files'].items():
        verify_digest(plans_root, relative, expected)

    routes = read(APP / 'editions/R03/mission.json')
    revision = canonical({k: v for k, v in routes.items() if k != 'version'})
    for task, edition in [('spray', 'R10/spray'), ('inspection', 'R08'), ('mowing', 'R10/mowing')]:
        mission = read(APP / 'editions' / edition / 'mission.json')
        if mission['source_plan_version'] != revision or routes['version'] != revision:
            raise ValueError(f'{task} source route revision differs')
        if mission['map_revision'] != atlas['revision'] or mission['map_id'] != atlas['map_id']:
            raise ValueError(f'{task} operational map differs')
    return dict(frozen_files=len(frozen['files']), source_database=frozen['source_database_sha256'],
                atlas_regions=sorted(atlas['regions']), original_plan_files=len(plan_files['files']),
                task_plan_revision=revision, rig_contracts=outputs)


def plans():
    from verify_r03 import validate as validate_routes
    from safe_plan_r10 import validate as validate_mowing
    from common_r1 import canonical

    editions = APP / 'editions'
    result = validate_routes(read(editions / 'R03/mission.json'))
    if not result['passed']:
        raise ValueError(f"Route geometry failed: {[k for k, v in result['checks'].items() if not v]}")
    safe = read(editions / 'R10/mowing_safe_plan.json')
    if canonical({k: v for k, v in safe.items() if k != 'revision'}) != safe['revision']:
        raise ValueError('Safe mowing path revision differs')
    verify_digest(editions / 'R09', 'mission.json', safe['source_sha256'])
    mowing = validate_mowing(safe, read(editions / 'R09/mission.json'))
    return dict(route_geometry=result['checks'], routes=result['plans'], safe_mowing=mowing)


def native():
    from safe_plan_r10 import load_checked

    safe = load_checked()
    lock = read(APP / 'config/shutdown.json')
    shim = next((lib for lib in lock['libraries'] if lib['path'] == lock['preload']), None)
    if shim is None:
        raise ValueError('No shutdown library entry for the locked preload')
    verify_digest(ROOT, lock['source']['path'], lock['source']['sha256'])
    system_hashes = {}
    for library in lock['libraries']:
        if library['path'] == shim['path']:
            continue
        absolute = Path(library['path'])
        verify_digest(Path('/'), absolute.relative_to('/'), library['sha256'])
        system_hashes[str(absolute)] = library['sha256']
    candidate = APP / 'build/candidate' / Path(shim['path']).name
    verify_digest(candidate.parent, candidate.name, shim['sha256'])
    return dict(ompl_binary_sha256=safe['backend_sha256'], safe_plan_revision=safe['revision'],
                shutdown_candidate_sha256=shim['sha256'], system_library_sha256=system_hashes)


def runtime():
    from common import check

    from lifecycle_r1 import validate as check_shutdown_abi
    from registry_r1 import load_task

    issues = []
    python = BASE / 'venvs/scene/bin/python'
    if os.environ.get('ROS_DISTRO') != 'jazzy':
        issues.append('First source optimization_v2/env.sh in this shell.')
    if python.is_file():
        imports = ('import rclpy, numpy, scipy, shapely, imageio_ffmpeg, PIL; '
                   'from mission_interfaces.action import ToolTask')
        result = subprocess.run([str(python), '-c', imports], capture_output=True, text=True, timeout=30)
        if result.returncode:
            lines = result.stderr.strip().splitlines()
            issues.append(lines[-1] if lines else f'Scene Python import exited {result.returncode}')
    else:
        issues.append(f'Missing scene Python: {python}. See BUILDING.md.')
    # The project starts these executables directly; each path matches its launch entry point.
    nodes = ('nav2_map_server/map_server', 'nav2_planner/planner_server',
             'nav2_controller/controller_server', 'nav2_bt_navigator/bt_navigator',
             'nav2_lifecycle_manager/lifecycle_manager', 'ros_gz_bridge/parameter_bridge',
             'robot_localization/ekf_node', 'rtabmap_odom/icp_odometry')
    for node in nodes:
        path = Path('/opt/ros/jazzy/lib') / node
        if not os.access(path, os.X_OK):
            issues.append(f'Missing ROS executable: {path}')
    profile_versions = {}
    for task in TASKS:
        try:
            profile_versions[task] = load_task(task)['version']
        except (ValueError, OSError) as exc:
            issues.append(f'{task} profile: {exc}')
    try:
        check_shutdown_abi()
    except (ValueError, OSError) as exc:
        issues.append(str(exc))
    result = check('R09')
    if not result['passed']:
        issues.append('R09 recording predecessor: ' + json.dumps(result))
    return dict(scene_python=str(python), ros_nodes_checked=len(nodes),
                task_profiles=profile_versions, issues=issues)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('assets', 'plans', 'native', 'runtime'))
    mode = parser.parse_args().mode
    try:
        details = {'assets': assets, 'plans': plans, 'native': native, 'runtime': runtime}[mode]()
    except (ImportError, OSError, ValueError, KeyError, subprocess.SubprocessError) as error:
        print(json.dumps(dict(check=mode, passed=False, error=str(error)), ensure_ascii=False))
        return 2
    passed = not details.get('issues')
    print(json.dumps(dict(check=mode, passed=passed, details=details), ensure_ascii=False, indent=2))
    return 0 if passed else 2


if __name__ == '__main__':
    raise SystemExit(main())
