"""Capture-aligned replay of the measured IMU/encoder/stop contract for the warm B1 hold."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

from stationary_gyro_r10 import StationaryYawBias


def rows(path):
    with path.open() as source:
        for line in source:
            yield json.loads(line)


def sha(path):
    with path.open('rb') as source:
        return hashlib.file_digest(source, 'sha256').hexdigest()


def measured_imus(path):
    pending = {}
    for row in rows(path):
        if row.get('sensor') != 'imu':
            continue
        t = row['t']
        if row['event'] == 'generated':
            pending[t] = row['observed_z']
        elif row['event'] == 'published':
            observed = pending.pop(t, None)
            if observed is not None:
                yield t, observed
        else:
            pending.pop(t, None)


def replay(run):
    paths = [run / name for name in ('generation_motion/generation.jsonl', 'inputs/inputs.jsonl',
             'inputs/result.json', 'icp_gate/quality.jsonl', 'control/gate.jsonl', 'spec.json')]
    gyro, inputs, adapter_result, icp, gate, session = paths
    startup = json.loads(adapter_result.read_text())['gyro_bias_estimate'][2]
    channels = [iter(row for row in rows(inputs) if row['key'] == 'wheel'), rows(icp), rows(gate)]
    heads = [next(stream, None) for stream in channels]
    tracker = StationaryYawBias()
    slipping, quality = False, {}
    counts = Counter()
    intervals = [(1122., 1132.), (1132., 1142.), (1142., 1152.), (1152., 1162.)]
    residuals = [[] for _ in intervals]
    moving = []
    for t, observation in measured_imus(gyro):
        counts['delivered_imu'] += 1
        for i, stream in enumerate(channels):
            while heads[i] and heads[i]['t'] <= t:
                row = heads[i]
                if i == 0:
                    tracker.observe_wheel(row['t'], row['v'], row['w'])
                    slipping = row.get('slip_suspected', False)
                elif i == 1:
                    quality = row
                else:
                    tracker.observe_gate(row)
                heads[i] = next(stream, None)
        if t < 4.:
            continue
        raw = observation - startup
        count = tracker.updates
        offset = tracker.update(t, raw, slipping or tracker.wheel_disputed(t, quality))
        wheel = tracker.previous(tracker.wheels, t, .08)
        gate_sample = tracker.previous(tracker.gates, t, .15)
        if tracker.updates > count:
            counts['learned'] += 1
            if not gate_sample or not gate_sample[1] or not wheel or abs(wheel[1]) > .012 or abs(wheel[2]) > .010:
                counts['learned_without_stationary_evidence'] += 1
        if not wheel:
            continue
        if abs(wheel[1]) > .08 and abs(wheel[2]) > .04:
            moving.append((abs(raw - offset - wheel[2]), (raw-offset)*wheel[2] > 0))
        for interval, samples in zip(intervals, residuals):
            if interval[0] <= t < interval[1]:
                samples.append((raw-wheel[2], raw-offset-wheel[2], offset, wheel[2]))
    windows = []
    for interval, samples in zip(intervals, residuals):
        original, candidate, offset, wheel = zip(*samples)
        windows.append(dict(interval_s=interval, count=len(samples), original_gyro_minus_wheel=statistics.mean(original),
                            candidate_gyro_minus_wheel=statistics.mean(candidate), mean_estimated_offset=statistics.mean(offset),
                            observed_encoder_yaw_rate=statistics.mean(wheel)))
    checks = dict(full_delivered_stream=counts['delivered_imu'] > 200000, observed_stop_updates=counts['learned']>1000,
                  all_updates_with_measured_evidence=not counts['learned_without_stationary_evidence'],
                  warm_hold_drift_reduced=all(abs(w['candidate_gyro_minus_wheel']) < abs(w['original_gyro_minus_wheel'])/2
                                              for w in windows[1:]),
                  measured_turns_preserved=len(moving)>1000 and statistics.mean(ok for _,ok in moving)>.98)
    return dict(session=json.loads(session.read_text())['session'], passed=all(checks.values()), checks=checks,
                scope='All published sensor observations, capture-time aligned wheel/ICP/actuator gate. DDS arrival '
                      'jitter cannot be reconstructed. Component evidence only. No simulator pose or hidden bias used.',
                startup_bias_rad_s=startup, samples=counts, units='rad/s', stationary_windows=windows,
                moving_samples=len(moving), moving_same_sign_fraction=statistics.mean(ok for _,ok in moving),
                moving_median_encoder_disagreement_rad_s=statistics.median(d for d,_ in moving),
                source_sha256={str(p.relative_to(run)):sha(p) for p in paths},
                estimator_sha256=sha(Path(__file__).with_name('stationary_gyro_r10.py')), replay_sha256=sha(Path(__file__)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = replay(args.run)
    data = json.dumps(report, indent=2) + '\n'
    if args.output:
        args.output.write_text(data)
    print(data, end='')
    raise SystemExit(0 if report['passed'] else 2)
