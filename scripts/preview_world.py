#!/usr/bin/env python3
"""Bounded world/chassis preview, independent of development acceptance receipts.

Uses the real scene generator and chassis model. Does not drive, serve tools,
relocalize, or generate a stage PASS. Generated files stay in preview_runs/.
"""
import argparse
import datetime
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from xml.etree import ElementTree as ET


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def main():
    default = next((p for p in Path(__file__).resolve().parents if (p/'optimization_v2').is_dir()), None)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=default)
    parser.add_argument('--vehicle', choices=['diff', 'ackermann'], default='diff')
    parser.add_argument('--seconds', type=float, default=90, help='Wall-clock preview limit, not task time')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--prepare-only', action='store_true')
    args = parser.parse_args()
    if args.root is None or not 1 <= args.seconds <= 600:
        parser.error('Existing project root and 1–600 wall seconds required')
    root = args.root.resolve()
    base = root/'optimization_v2'
    scene_python=base/'venvs/scene/bin/python'
    if Path(sys.prefix).resolve() != (base/'venvs/scene').resolve():
        if not scene_python.is_file():
            raise RuntimeError('Create the scene virtual environment as described in BUILDING.md')
        os.execv(str(scene_python),[str(scene_python),str(Path(__file__).resolve()),*sys.argv[1:]])
    lock_path = root/'native_20261108/runs/session.lock'
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    lock = lock_path.open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('A project simulation is already running; preview refused.', file=sys.stderr)
        return 2
    run_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())
    out = root/'preview_runs'/run_id
    out.mkdir(parents=True)
    scene_dir = base/'ws/src/scenario_world/scripts'
    scene = module('preview_scene', scene_dir/'generate_scene.py')
    chassis = module('preview_chassis', base/'ws/src/robot_description/scripts/generate.py')
    generated = scene.generate(out/'scene', seed=41)
    document = ET.parse(generated['world'])
    world = document.getroot().find('world')
    # Preview is geometry only. The actual S04–S15 runners own sensor qualification.
    probe = world.find("model[@name='validation_probe']")
    if probe is not None:
        world.remove(probe)
    cfg = chassis.config(args.vehicle)
    robot = chassis.vehicle(cfg, 'repository_preview_'+run_id)
    chassis.el(robot, 'pose', '30.5 15.3 .025 0 0 .15')
    for link in robot.findall('link'):
        for sensor in list(link.findall('sensor')):
            link.remove(sensor)
    world.append(robot)
    # No GPU camera is needed for this scene overview.
    for plugin in list(world.findall('plugin')):
        if plugin.get('name') == 'gz::sim::systems::Sensors':
            world.remove(plugin)
    world_path = out/'preview.sdf'
    chassis.write(world_path, document.getroot())
    for uri in document.findall('.//uri'):
        if uri.text.startswith('file://') and not Path(uri.text[7:]).is_file():
            raise FileNotFoundError(uri.text)
    result = dict(scope='SCENE_PREVIEW_ONLY', vehicle=args.vehicle, world=str(world_path),
                  wall_limit_s=args.seconds, navigation=False, acceptance_receipt=False)
    if args.prepare_only:
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
        return 0
    required = [base/'ws/install/scenario_world/lib/libs02_motion.so',
                base/'ws/install/chassis_bridge/lib/libs03_drive.so']
    missing = [str(p) for p in required if not p.is_file()]
    if missing:
        raise FileNotFoundError('Build scenario_world and chassis_bridge first: '+', '.join(missing))
    env = os.environ.copy()
    env['GZ_PARTITION'] = 'repository_preview_'+run_id
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(str(p.parent) for p in required)+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    children, streams = [], []
    started = time.monotonic()
    def launch(role, command):
        stream = (out/(role+'.log')).open('w')
        streams.append(stream)
        process = subprocess.Popen(command, cwd=root, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((role, process))
        return process
    def stop(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    try:
        server = launch('server', ['gz', 'sim', '--force-version', '8', '-s', '-r', '--seed', '41', str(world_path)])
        gui = None
        if not args.headless:
            # Use the same repository camera configuration as S02.
            sys.path.insert(0, str(scene_dir))
            runtime = module('preview_scene_runtime', scene_dir/'runtime.py')
            runtime.gui_config(out/'gui.config', 'overview')
            gui = launch('gui', ['gz', 'sim', '--force-version', '8', '-g', '--render-engine-gui', 'ogre', '--gui-config', str(out/'gui.config')])
        print('Scene preview; stationary chassis, scripted scene actors; no autonomy claim.', flush=True)
        print('Output: '+str(out), flush=True)
        while time.monotonic()-started < args.seconds:
            if server.poll() is not None:
                raise RuntimeError('Gazebo server exited during preview')
            if gui and gui.poll() is not None:
                break
            time.sleep(.2)
        result['status'] = 'PREVIEW_FINISHED'
    except KeyboardInterrupt:
        result['status'] = 'PREVIEW_STOPPED'
    finally:
        for role, process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for role, process in reversed(children):
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        for stream in streams:
            stream.close()
        result.update(wall_s=time.monotonic()-started,
                      exit_codes={role: process.returncode for role, process in children})
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
