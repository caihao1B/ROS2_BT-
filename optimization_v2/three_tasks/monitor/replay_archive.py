"""Export a self-contained, local-browser review of closed ROS/Gazebo runs."""
import argparse
from base64 import b64encode
from bisect import bisect_left
import gzip
from html import escape
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent
CHANNELS = {
    'bt': ('decisions.jsonl', ('action', 'reason_code', 'blocked_duration_s'), .5),
    'gate': ('control/gate.jsonl', ('mode', 'v', 'w', 'stopped', 'tool_stopped'), .35),
    'loc': ('reference/reference.jsonl', ('valid', 'reason', 'pose'), .5),
    'gnss': ('reference/reference.jsonl', ('gnss_reason', 'innovation_m'), .5),
    'risk': ('risk/risk.jsonl', ('valid', 'reason', 'max_speed'), .5),
    'wheel': ('inputs/inputs.jsonl', ('v', 'w', 'slip_suspected'), .35),
    'imu': ('inputs/inputs.jsonl', ('gyro', 'bias'), .35),
    'icp': ('icp_gate/quality.jsonl', ('valid', 'inlier_ratio', 'correspondences'), 1.5),
}
EVENT_FIELDS = ('t', 'event', 'reason', 'reason_code', 'error', 'terminal', 'state', 'point',
                'segment', 'direction', 'success', 'operation', 'confirmed_quantity', 'remaining_quantity',
                'stage', 'dispatch_id', 'next_point', 'residual_quantity', 'credited_quantity',
                'confirmed', 'remaining', 'meter_before', 'meter_after', 'replans', 'checks')


def records(path):
    if not path.is_file():
        return
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def bounded_samples(run, times):
    """Pick the latest eligible measurement <= the cursor; tolerate out-of-order packets."""
    output = [dict() for _ in times]
    for source in dict.fromkeys(spec[0] for spec in CHANNELS.values()):
        names = [name for name, spec in CHANNELS.items() if spec[0] == source]
        for row in records(run / source):
            observed = row.get('t')
            if not isinstance(observed, (int, float)) or not math.isfinite(observed):
                continue
            for name in names:
                if name in ('wheel', 'imu') and row.get('key') != name:
                    continue
                sample = row.get('native_supervisor', {}) if name == 'gnss' else row
                stamp = sample.get('t')
                if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or stamp > observed:
                    continue
                fields, maximum_age = CHANNELS[name][1:]
                for i in range(bisect_left(times, observed), len(times)):
                    if times[i] - stamp > maximum_age:
                        break
                    current = output[i].get(name)
                    if current is None or current[0] < stamp:
                        output[i][name] = [stamp, *[sample.get(f) for f in fields]]
    return output


def read(run, name):
    return json.loads((run / name).read_text(encoding='utf-8'))


def dataset(run):
    runtime = read(run, 'runtime.json')
    mission = read(run, 'mission_result.json')
    spec = read(run, 'spec.json')
    analysis = next((read(run, f) for f in ('analysis_mowing_r10.json', 'analysis_spray_r10.json',
                     'analysis_r08.json', 'analysis_r09.json') if (run / f).is_file()), {})
    events = [{k: r[k] for k in EVENT_FIELDS if k in r}
              for r in records(run / 'mission_events.jsonl') if isinstance(r.get('t'), (float, int))]
    end = mission.get('t')
    if not isinstance(end, (float, int)) or end <= 0:
        raise ValueError(f'{run.name}: missing mission end stamp')
    events = [e for e in events if 0 <= e['t'] <= end]
    times = sorted({*range(0, math.ceil(end), 5), end, *(e['t'] for e in events)})
    behavior = None
    receipt = run / 'candidate_acceptance.json'
    if receipt.is_file():
        candidate = read(run, receipt.name)
        if candidate['session'] != run.name or candidate['formal_credit'] is not False:
            raise ValueError(f'{run.name}: isolated behavior receipt does not match the source run')
        behavior = dict(result=candidate['behavior_result'], confirmed=len(candidate['completed']),
                        selected=len(candidate['selected']), original=candidate['original_count'],
                        coarse=candidate['coarse_defer_demonstrated'], precision=candidate['precision_defer_demonstrated'])
    handoff_receipt = run / 'candidate_handoff_acceptance.json'
    handoff = None
    if handoff_receipt.is_file():
        result = read(run, handoff_receipt.name)
        if result['session'] != run.name or result['formal_credit'] is not False:
            raise ValueError(f'{run.name}: handoff receipt and session differ')
        handoff = dict(confirmed=len(result['completed']), selected=len(result['selected']),
                       original=mission['original_points'], outstanding=mission.get('failed', []),
                       task_passed=mission['passed'], status=result.get('handoff_status',result.get('terminal')),
                       behavior_passed=result.get('passed'))
    isolated_mowing = None
    mower_receipt = run / 'candidate_analysis.json'
    if spec['task']=='mowing' and spec.get('candidate_profile') and mower_receipt.is_file():
        score = read(run, mower_receipt.name)
        isolated_mowing = dict(passed=score['passed'], completed=len(mission['completed_segments']),
                               selected=mission['selected_segments'], terminal=mission.get('terminal'))
    return dict(id=run.name, task=spec['task'], case=spec['case'], profile=spec.get('candidate_profile'),
                time=times, behavior=behavior, handoff=handoff, isolated_mowing=isolated_mowing,
                observations=bounded_samples(run, times), events=events,
                end=dict(passed=analysis.get('passed'), terminal=mission.get('terminal'),
                         error=mission.get('error'), runtime_clean=runtime.get('passed'),
                         confirmed_area=analysis.get('selected_confirmed_union_m2'),
                         residual_area=analysis.get('selected_residual_m2')))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    data = dict(channels={k: v[1] for k, v in CHANNELS.items()}, runs=[dataset(p) for p in args.runs])
    raw = json.dumps(data, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode()
    packed = b64encode(gzip.compress(raw, compresslevel=6, mtime=0)).decode('ascii')
    page = (BASE / 'replay_template.html').read_text(encoding='utf-8')
    page = page.replace('ARCHIVE_BASE64', packed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(page, encoding='utf-8')
    for r in data['runs']:
        print(escape(r['id']), len(r['time']), 'frames', len(r['events']), 'events')


if __name__ == '__main__':
    main()
