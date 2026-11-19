#!/usr/bin/env python3
"""Reproduce the stopped first-gear passage audit, with separate onboard and evaluation inputs."""
import argparse
import bisect
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path
import statistics

from arrival_mowing_r10 import stable_arrival

SESSION = 'mowing_20261119_090706_626159'
T0, T1 = 110.64, 114.53
STREAMS = {
    'loc': 'reference/reference.jsonl', 'gate': 'control/gate.jsonl',
    'bt': 'decisions.jsonl', 'risk': 'risk/risk.jsonl',
    'tool': 'tool_observer/observations.jsonl', 'source': 'global_sources/sources.jsonl',
    'inputs': 'inputs/inputs.jsonl', 'events': 'mission_events.jsonl',
    'truth': 'evaluation/truth.jsonl', 'physics': 'tool_evaluation/physical.jsonl'}
CONFIGS = {'mission': 'mission.json', 'spec': 'spec.json',
           'tool_config': 'scene/tool.json', 'route': 'scene/qualification_transfer.json'}


def capture(run):
    data = {'source_sha256': {}, 'rows': {}, 'configuration': {}}
    for name, rel in CONFIGS.items():
        raw = (run / rel).read_bytes()
        data['source_sha256'][rel] = hashlib.sha256(raw).hexdigest()
        data['configuration'][name] = json.loads(raw)
    # Keep only the documented contract fields in the portable excerpt. Full source digests
    # still bind the entire on-disk mission, tool, candidate and route manifests.
    cfg = data['configuration']
    fields = {
        'mission': ('version', 'map_revision', 'map_offset', 'motion_observation_age_s'),
        'spec': ('session', 'rig_revision'),
        'tool_config': ('tool_revision', 'stopped_max_rad_s', 'feedback_age_sim_s')}
    arrival = cfg['mission']['navigation_execution']['stopped_arrival_confirmation']
    for name, keys in fields.items():
        cfg[name] = {key: cfg[name][key] for key in keys}
    cfg['mission']['navigation_execution'] = {'stopped_arrival_confirmation': arrival}
    segment = cfg['route']['segments'][0]
    cfg['route'] = {'revision': cfg['route']['revision'], 'segments': [
        {'id': segment['id'], 'poses': [segment['poses'][-1]]}]}
    for key, rel in STREAMS.items():
        digest = hashlib.sha256()
        rows = []
        with (run / rel).open('rb') as stream:
            for line in stream:
                digest.update(line)
                item = json.loads(line)
                if 109.0 <= item['t'] <= 116.5 or (key == 'events' and 89.0 <= item['t'] < 109.0):
                    if key != 'inputs' or item.get('key') == 'imu':
                        rows.append(item)
        data['source_sha256'][rel] = digest.hexdigest()
        data['rows'][key] = rows
    return data


class Past:
    """Latest sample captured at or before t. Forward samples cannot satisfy a missing input."""
    def __init__(self, rows):
        self.rows = sorted(rows, key=lambda r: r['t'])
        self.times = [r['t'] for r in self.rows]

    def at(self, t, age):
        index = bisect.bisect_right(self.times, t) - 1
        return self.rows[index] if index >= 0 and t - self.times[index] <= age else {}


def bounds(values):
    return [min(values), max(values)] if values else None


def yaw_error(angle, goal):
    return abs(math.remainder(angle - goal, math.tau))


def stopped_motor(feedback, spec, tool_config):
    keys = ('t', 'rpm_rad_s', 'flow', 'request_on')
    return (all(k in feedback for k in keys) and feedback.get('healthy')
            and feedback.get('sensor_epoch') == spec['session']
            and feedback.get('tool_revision') == tool_config['tool_revision']
            and not feedback['request_on'] and feedback['flow'] == 0
            and abs(feedback['rpm_rad_s']) < tool_config['stopped_max_rad_s'])


def observed_windows(config, observations):
    mission, spec, tool_config = (config[k] for k in ('mission', 'spec', 'tool_config'))
    segment = config['route']['segments'][0]
    goal = segment['poses'][-1]
    protocol = mission['navigation_execution']['stopped_arrival_confirmation']
    gate_t, bt_t, risk_t = (Past(observations[k]) for k in ('gate', 'bt', 'risk'))
    tool_t = Past([r for r in observations['tool'] if r['kind'] == 'feedback'])
    context_t = Past([r for r in observations['tool'] if r['kind'] == 'context'])
    found, first, samples = [], None, []
    for ref in observations['loc']:
        now = ref['t']
        if not T0 <= now <= T1:
            continue
        gate, bt, risk = gate_t.at(now, .15), bt_t.at(now, .3), risk_t.at(now, mission['motion_observation_age_s'])
        feedback = tool_t.at(now, tool_config['feedback_age_sim_s'])
        context = context_t.at(now, .15)
        pose = ref.get('pose', [])
        evidence = bt.get('evidence', {})
        native = ref.get('native_supervisor', {})
        consistent = (
            len(pose) == 3 and ref.get('valid') and not ref.get('truth_used', True)
            and native.get('navigation_allowed') and not native.get('plan_revalidation_required', True)
            and 0 <= now - ref.get('odom_t', -1) <= .15
            and ref.get('map_revision') == mission['map_revision']
            and ref.get('rig_revision') == spec['rig_revision']
            and bt.get('session') == spec['session'] and bt.get('task_version') == mission['version']
            and evidence.get('bindings_valid') and not evidence.get('stale', True)
            and bt.get('action') == 'EXECUTE' and not bt.get('manual_latched', True)
            and risk.get('valid') and risk.get('boundary_valid')
            and risk.get('localization_epoch') == ref.get('localization_epoch')
            and context.get('segment_id') == segment['id'] and context.get('segment') == 'TURN_TOOL_OFF'
            and gate.get('mode') == 'STOP' and gate.get('stopped') and gate.get('tool_stopped')
            and stopped_motor(feedback, spec, tool_config))
        if not consistent:
            samples.clear()
            continue
        xy = math.dist(pose[:2], goal[:2])
        yaw = yaw_error(pose[2], goal[2])
        if xy > protocol['xy_m'] or yaw > protocol['yaw_rad']:
            samples.clear()
            continue
        stamp = ref['odom_t']
        if not samples or stamp > samples[-1][0]:
            samples.append((stamp, pose))
        samples = [s for s in samples if stamp - s[0] <= .75]
        if first is None and stable_arrival(samples, goal, protocol['xy_m'], protocol['yaw_rad']):
            first = {'t': now, 'unique_odometry_stamps': len(samples), 'span_s': stamp - samples[0][0]}
        found.append({'t': now, 'xy': xy, 'yaw': yaw, 'gate': gate, 'tool_seq': feedback['seq']})
    return goal, found, first


def projected_variance(cov):
    # Largest eigenvalue of the 2x2 horizontal block (3x3 row-major covariance).
    a, b, d = cov[0], .5 * (cov[1] + cov[3]), cov[4]
    if not all(math.isfinite(v) for v in (a, b, d)) or min(a, d) < 0 or a * d < b * b:
        raise ValueError('Invalid horizontal GNSS covariance')
    return .5 * (a + d + math.hypot(a - d, 2 * b))


def rotated_lever(lever, roll, pitch, yaw):
    x, y, z = lever
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    u, v = cp*x + sp*(sr*y + cr*z), cr*y - sr*z
    return [cy*u - sy*v, sy*u + cy*v]


def gnss_readback(goal, selected, observations, xy_limit):
    """No ground truth, action commands or positive endpoint authorization in this calculation."""
    loc, imus = Past(observations['loc']), Past(observations['inputs'])
    fixes = []
    for fix in observations['source']:
        t = fix['t']
        if not (selected[0]['t'] <= t <= selected[-1]['t'] and fix.get('kind') == 'gnss'
                and fix.get('valid') and fix.get('mode') == 'fixed'):
            continue
        ref, imu = loc.at(t, .15), imus.at(t, .15)
        if not ref.get('valid') or not ref.get('pose'):
            continue
        yaw = ref['pose'][2]
        lever, antenna = fix['antenna_base_m'], fix['antenna']
        flat = rotated_lever(lever, 0., 0., yaw)
        flat_xy = [antenna[i] - flat[i] for i in range(2)]
        sigma = math.sqrt(projected_variance(fix['covariance']))
        radius = math.sqrt(ref['native_supervisor']['nis_limit']) * sigma
        tilted_distance = None
        if imu.get('tilt_valid'):
            tilted = rotated_lever(lever, *imu['tilt'], yaw)
            tilted_distance = math.dist([antenna[i] - tilted[i] for i in range(2)], goal[:2])
        fixes.append({'t': t, 'flat_base_xy': flat_xy, 'flat_xy_error_m': math.dist(flat_xy, goal[:2]),
                      'imu_tilt_valid': bool(imu.get('tilt_valid')), 'tilted_xy_error_m': tilted_distance,
                      'gnss_horizontal_sigma_max_m': sigma, 'model_radius_m': radius,
                      'tilted_distance_plus_gnss_radius_m': None if tilted_distance is None else tilted_distance + radius})
    flat_errors = [r['flat_xy_error_m'] for r in fixes]
    corrected = [r['tilted_xy_error_m'] for r in fixes if r['tilted_xy_error_m'] is not None]
    upper_radii = [r['tilted_distance_plus_gnss_radius_m'] for r in fixes if r['tilted_distance_plus_gnss_radius_m'] is not None]
    return {'fixed_samples': len(fixes), 'flat_error_median_m': statistics.median(flat_errors) if flat_errors else None,
            'flat_count_inside_xy': sum(d <= xy_limit for d in flat_errors),
            'tilt_valid_samples': len(corrected), 'tilted_error_range_m': bounds(corrected),
            'tilted_error_median_m': statistics.median(corrected) if corrected else None,
            'tilted_plus_gnss_radius_range_m': bounds(upper_radii),
            'tilted_fixes_whose_reported_gnss_error_circle_fits': sum(d <= xy_limit for d in upper_radii),
            'model': 'NIS coefficient from each live reference times largest horizontal GNSS covariance eigenvalue; square root gives conservative circular envelope of the reported horizontal ellipse. Uses each fixed observation separately with no averaging discount. Attitude, datum and lever-arm systematic uncertainty are not certified by this bound. Shared map yaw and receipt-time uncertainty preclude independent absolute 3D certification.',
            'onboard_positive_authority': False, 'fixes': fixes}


def independent_score(rows, goal, selected, config):
    truth, physical = Past(rows['truth']), Past(rows['physics'])
    offset = config['mission']['map_offset']
    xy_errors, angles, body_speed = [], [], []
    tools_stopped = 0
    for obs in selected:
        t = obs['t']
        measured, rotor = truth.at(t, .15), physical.at(t, .15)
        if measured:
            xy = [measured['xyz'][i] - offset[i] for i in range(2)]
            xy_errors.append(math.dist(xy, goal[:2]))
            angles.append(yaw_error(measured['rpy'][2], goal[2]))
        if rotor:
            body_speed.append(rotor['body_v'])
            tools_stopped += bool(rotor['healthy'] and not rotor['on'] and rotor['flow'] == 0
                                  and abs(rotor['rpm_rad_s']) < config['tool_config']['stopped_max_rad_s'])
    limits = config['mission']['navigation_execution']['stopped_arrival_confirmation']
    return {'truth_scored': len(xy_errors), 'truth_xy_error_range_m': bounds(xy_errors),
            'truth_yaw_error_range_rad': bounds(angles), 'truth_both_limits_passed': sum(
                x <= limits['xy_m'] and a <= limits['yaw_rad'] for x, a in zip(xy_errors, angles)),
            'physical_tool_off_frames': tools_stopped, 'physical_body_speed_range_mps': bounds(body_speed)}


def audit(payload):
    rows, cfg = payload['rows'], payload['configuration']
    if cfg['spec']['session'] != SESSION:
        raise ValueError(f'This diagnostic is bound to the observed {SESSION} first-gear window')
    onboard = {k: v for k, v in rows.items() if k not in ('truth', 'physics')}
    goal, sample, first = observed_windows(cfg, onboard)
    if not sample:
        raise ValueError('No eligible stopped observation in the specified original endpoint window')
    gates = [s['gate'] for s in sample]
    limits = cfg['mission']['navigation_execution']['stopped_arrival_confirmation']
    readback = gnss_readback(goal, sample, onboard, limits['xy_m'])
    return {'schema': 'R10-OFFLINE-ENDPOINT-READBACK/1', 'session': cfg['spec']['session'],
            'source_sha256': payload['source_sha256'], 'task_version': cfg['mission']['version'],
            'transfer_revision': cfg['route']['revision'], 'segment': cfg['route']['segments'][0]['id'],
            'map_goal': goal, 'checked_interval_s': [T0, T1], 'original_arrival_limits': limits,
            'onboard': {'samples': len(sample), 'time_range_s': [sample[0]['t'], sample[-1]['t']],
                        'estimated_xy_error_range_m': bounds([s['xy'] for s in sample]),
                        'estimated_yaw_error_range_rad': bounds([s['yaw'] for s in sample]),
                        'stable_pose_window_first': first, 'distinct_tool_readbacks': len({s['tool_seq'] for s in sample}),
                        'nav_idle_frames': sum(g['nav_idle'] for g in gates),
                        'command_gate_reason_counts': dict(Counter('|'.join(g['reasons']) for g in gates)),
                        'armed_directions': sorted({g['armed_direction'] for g in gates}),
                        'raw_navigation_linear_velocity_range_mps': bounds([g['raw_nav_v'] for g in gates]),
                        'action_events': onboard['events']},
            'onboard_gnss_readback': readback,
            'independent_physical_audit_after_selection': independent_score(rows, goal, sample, cfg),
            'disposition': 'Read-only evaluation. No endpoint, gear, work or area credit. The active forward-gear navigation request was clipped for negative speed; an estimated stop from clipping cannot confirm a settled endpoint.',
            'time_alignment': 'Latest causal sample in recorded simulation timestamps. Captures without an application receipt clock do not establish an exact callback delivery order.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='original run directory or gzip-compressed input excerpt')
    parser.add_argument('--excerpt', type=Path, help='write reproducible compressed excerpt for offline review')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.source.is_file():
        with gzip.open(args.source, 'rt', encoding='utf-8') as f:
            payload = json.load(f)
    else:
        payload = capture(args.source)
    if args.excerpt:
        raw = json.dumps(payload, separators=(',', ':'), ensure_ascii=False).encode()
        args.excerpt.write_bytes(gzip.compress(raw, mtime=0))
    report = json.dumps(audit(payload), indent=2, ensure_ascii=False) + '\n'
    if args.output:
        args.output.write_bytes(report.encode('utf-8'))
    else:
        print(report, end='')


if __name__ == '__main__':
    main()
