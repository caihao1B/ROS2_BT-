#!/usr/bin/env python3
"""Replay an A1 session into disjoint per-obligation time accounts, using no ROS processes."""
import argparse
import csv
import gzip
import hashlib
import io
import json
import math
import sqlite3
from collections import Counter
from pathlib import Path

PROJECT = Path(__file__).resolve().parents[2]
DEFAULT_TRACE = PROJECT / 'validation/r10_bounded_20261119/a1/phase_observations.json.gz'
PHASES = ('travel_s', 'precision_s', 'localization_wait_s', 'spray_flow_s', 'other_s')
COMMANDS = ('still', 'rotate', 'slow', 'medium', 'cruise')
MOTION = ('translating', 'rotating', 'stationary')
COLUMNS = ('ordinal', 'point_id', 'status', 'start_sim_s', 'end_sim_s', 'elapsed_s', *PHASES,
           *(f'navigation_{key}_s' for key in COMMANDS), *(f'body_{key}_during_nav_s' for key in MOTION),
           'nav_attempts', 'requested_dose', 'ledger_dose')
TICKS = 1_000_000
PRECISION_EVENTS = {'NAV_SUCCESS', 'GLOBAL_APPROACH_STOP_FOR_PRECISION', 'LOCAL_DOCK_GOAL',
                    'LOCAL_DOCK_PROPOSAL_REFUSED', 'DOCKING_STABLE_OFFSET', 'DOCKING_OBSERVATION_CONTINUED',
                    'LOCAL_DOCK_NAV_SUCCESS', 'LOCAL_DOCK_NAV_FAILED', 'LOCAL_DOCK_STOP_FOR_GLOBAL_OBSERVATION'}
HOLD_EVENTS = {'LATE_LOCALIZATION_HOLD_DISPATCHED', 'STOPPED_RELOCALIZATION_REQUEST',
               'LOCALIZATION_PREVIEW_PREPARED', 'STOPPED_PLAN_REVALIDATION', 'LOCALIZATION_PLAN_COMMITTED'}


def load(path):
    return json.loads(path.read_bytes())


def tick(t):
    if not math.isfinite(t) or t < 0:
        raise ValueError('Invalid sim timestamp')
    return round(t * TICKS)


def secs(t):
    return t / TICKS


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def command_mode(v, w):
    if abs(v) < .015:
        return 'rotate' if abs(w) > .03 else 'still'
    if abs(v) < .10:
        return 'slow'
    return 'medium' if abs(v) < .25 else 'cruise'


def sample_changes(path, classify, *, interval_ends=False):
    """Run-length encode sample states. Retain the full sample span and gap for audit."""
    changes, previous, previous_t, count, gap = [], None, None, 0, 0
    with path.open() as stream:
        for line in stream:
            sample = json.loads(line)
            t = tick(sample['t'])
            if previous_t is not None:
                if t < previous_t:
                    raise ValueError(f'Sensor clock regressed in {path.name}')
                gap = max(gap, t - previous_t)
            state = classify(sample)
            # The physical plugin derives body velocity at t from displacement since
            # the previous sample. Attribute that measurement to the preceding interval.
            if interval_ends:
                if previous_t is not None and (not changes or state != previous):
                    changes.append([previous_t, state])
            elif state != previous or not count:
                changes.append([t, state])
            previous, previous_t = state, t
            count += 1
    if count < 2 or gap > tick(.15):
        raise ValueError(f'Missing or sparse samples in {path.name}')
    return {'samples': count, 'first_us': changes[0][0], 'last_us': previous_t,
            'max_gap_us': gap, 'changes': changes}


def capture(run):
    mission, result, spec = [load(run / name) for name in ('mission.json', 'mission_result.json', 'spec.json')]
    cluster = next(c for c in mission['clusters'] if c['id'] == 'A1')
    if (spec['task'], spec['cluster'], result['cluster']) != ('spray', 'A1', 'A1'):
        raise ValueError('Expected an A1 spray session')
    selected = spec.get('candidate_selected_point_ids')
    if selected is None:
        omitted = set(result['not_selected'])
        selected = [p['id'] for p in cluster['points'] if p['id'] not in omitted]
    points = [(p['id'], p['quantity']) for p in cluster['points']]
    if len(selected) != result['selected_points'] or len(points) != result['original_points']:
        raise ValueError('Selected service count mismatch')
    if spec.get('candidate_mission_version', mission['version']) != mission['version']:
        raise ValueError('Configured mission version differs from the evidence')
    events = []
    with (run / 'mission_events.jsonl').open() as stream:
        for line in stream:
            entry = json.loads(line)
            events.append({k: entry[k] for k in ('t', 'event', 'point', 'state', 'remaining', 'stable',
                                'uncertainty', 'replayed', 'reason', 'confirmed') if k in entry})
            if 'continuous_stopped_s' in entry.get('stability', {}):
                events[-1]['continuous_stopped_s'] = entry['stability']['continuous_stopped_s']

    def physical_state(sample):
        # Positive independent discharge, even if a cancellation has already been sent.
        key = sample['key'] if sample['flow'] > 0 else None
        if sample['flow'] > 0 and not key:
            raise ValueError('Discharge occurred without an active work key')
        if key and not key.startswith(mission['version'] + ':'):
            raise ValueError('Discharge was attributed to another mission version')
        return key.rsplit(':', 1)[-1] if key else None

    physical_file = run / 'tool_evaluation/physical.jsonl'
    flow = sample_changes(physical_file, physical_state)
    previous = None

    def body_state(sample):
        nonlocal previous
        angular = 0.
        if previous is not None and sample['t'] > previous['t']:
            dyaw = sample['tool_yaw'] - previous['tool_yaw']
            angular = abs(math.atan2(math.sin(dyaw), math.cos(dyaw))) / (sample['t'] - previous['t'])
        previous = sample
        if abs(sample['body_v']) >= .015:
            return 'translating'
        return 'rotating' if angular > .03 else 'stationary'

    body = sample_changes(physical_file, body_state, interval_ends=True)
    cmd = sample_changes(run / 'control/gate.jsonl', lambda r: command_mode(r['v'], r['w']))
    db = (run / 'ledger/ledger.sqlite').resolve()
    with sqlite3.connect(db.as_uri() + '?mode=ro', uri=True) as conn:
        confirmed = []
        for key, status, qty, uncertain in conn.execute('SELECT key,status,quantity,uncertain FROM jobs'):
            if not key.startswith(mission['version'] + ':'):
                raise ValueError('Tool job from another mission')
            if status == 'SUCCEEDED':
                if uncertain:
                    raise ValueError('Uncertain successful ledger entry')
                confirmed.append([key.rsplit(':', 1)[-1], qty])
    files = ('mission_events.jsonl', 'spec.json', 'mission.json', 'mission_result.json', 'tool_evaluation/physical.jsonl',
             'control/gate.jsonl', 'ledger/ledger.sqlite', 'controller_server.log')
    with (run / 'controller_server.log').open() as stream:
        warnings = sum('RegulatedPurePursuitController detected collision ahead!' in line for line in stream)
    return dict(schema='A1-PHASE-OBSERVATIONS/1', session=spec['session'], mission_version=mission['version'],
                source_sha256={name: digest(run / name) for name in files}, points=points, selected=selected,
                confirmed_ids=[row['point'] for row in result['completed']], ledger=sorted(confirmed),
                cutoff_sim_s=result['t'], terminal_error=result.get('error'), mission_passed=result['passed'],
                events=events, flow=flow, commands=cmd, body=body, controller_collision_warnings=warnings)


def mission_markers(events):
    markers = []
    retry_start = None
    for e in events:
        t, name, reason = tick(e['t']), e['event'], e.get('reason', '')
        stage = None
        if name == 'NAV_GOAL':
            stage = 'travel_s'
        elif name in PRECISION_EVENTS:
            stage = 'precision_s'
        elif name == 'DOCKING_SETTLED_CHECK':
            if retry_start is not None and 'continuous_stopped_s' in e:
                # After RESIDUAL_RETRY the executor waits for permission and then docks.
                # Attribute only the observed continuous stationary interval to that retry.
                start = max(retry_start, t - tick(e['continuous_stopped_s']))
                markers.append((start, 'precision_s'))
            retry_start = None
            stage = 'other_s' if e['stable'] else 'precision_s'
        elif name in ('BT_INTERRUPTS_NAV', 'BT_CANCEL_TOOL', 'LOCAL_DOCK_INTERRUPTED'):
            stage = 'localization_wait_s' if 'LOCALIZATION' in reason else 'other_s'
        elif name in HOLD_EVENTS:
            stage = 'localization_wait_s'
        elif name == 'RESIDUAL_RETRY':
            retry_start, stage = t, 'other_s'
        elif name in ('NAV_TERMINAL_FAILURE', 'NAV_REJECTED', 'TOOL_RESULT', 'TASK_STOP', 'SUBSET_FINISHED',
                      'LOCALIZATION_REVALIDATED_RESUME', 'NEAR_GOAL_PROGRESS_REPLAN'):
            stage = 'other_s'
        if stage:
            markers.append((t, stage))
    return markers


def point_boundaries(trace):
    events = trace['events']
    times = [tick(e['t']) for e in events]
    if times != sorted(times) or any(t > tick(trace['cutoff_sim_s']) for t in times):
        raise ValueError('Mission timestamps are out of sequence')
    goals, success = {}, {}
    for e in events:
        p, name, t = e.get('point'), e['event'], tick(e['t'])
        if name == 'NAV_GOAL':
            goals.setdefault(p, t)
        elif name == 'TOOL_RESULT' and e['state'] == 'SUCCEEDED' and not e['replayed']:
            if p in success or e['remaining'] > 1e-9 or e['uncertainty']:
                raise ValueError('Repeated or uncertain service confirmation')
            success[p] = t
    ledger = dict(trace['ledger'])
    if (list(success) != trace['confirmed_ids'] or set(success) != set(ledger)
            or not set(success).issubset(goals)):
        raise ValueError('Event, result and persistent ledger disagree')
    selected = trace['selected']
    planned = [p for p, _ in trace['points']]
    if (len(set(planned)) != len(planned) or len(selected) != len(set(selected))
            or not set(selected).issubset(planned) or list(goals) != [p for p in selected if p in goals]):
        raise ValueError('Goal order differs from selected obligations')
    order = list(goals)
    first = goals[order[0]] if order else tick(trace['cutoff_sim_s'])
    stop = next((tick(e['t']) for e in events if e['event'] in ('TASK_STOP', 'SUBSET_FINISHED')),
                tick(trace['cutoff_sim_s']))
    boundaries = {}
    previous = first
    for i, p in enumerate(order):
        end = success.get(p, goals[order[i + 1]] if i + 1 < len(order) else stop)
        if not previous <= goals[p] <= end <= stop:
            raise ValueError('Invalid point-cycle boundaries')
        boundaries[p] = (previous, end)
        previous = end
    return boundaries, ledger


def analyze(trace):
    if trace['schema'] != 'A1-PHASE-OBSERVATIONS/1':
        raise ValueError('Unknown observation schema')
    bounds, ledger = point_boundaries(trace)
    totals = {p: Counter() for p in bounds}
    markers = [(t, 'phase', s) for t, s in mission_markers(trace['events'])]
    for p, (start, end) in bounds.items():
        markers.extend(((start, 'point', p), (end, 'point', None)))
    for channel in ('flow', 'commands', 'body'):
        sensor = trace[channel]
        if bounds and (sensor['first_us'] > min(lo for lo, _ in bounds.values())
                     or sensor['last_us'] < max(hi for _, hi in bounds.values())
                     or sensor['max_gap_us'] > tick(.15)):
            raise ValueError('Sensor evidence does not cover the assessed interval')
        markers.extend((t, channel, value) for t, value in sensor['changes'])
    state = {'point': None, 'phase': 'other_s', 'flow': None}
    last = 0
    for t, channel, value in sorted(markers, key=lambda m: m[0]):
        point = state['point']
        if t > last:
            flow = state['flow']
            if flow is not None and flow != point:
                raise ValueError('Physical discharge outside its own point cycle')
            if point is not None:
                phase = 'spray_flow_s' if flow is not None else state['phase']
                counts = totals[point]
                counts[phase] += t - last
                if phase == 'travel_s':
                    counts[f"navigation_{state['commands']}_s"] += t - last
                    counts[f"body_{state['body']}_during_nav_s"] += t - last
        last = t
        state[channel] = value
    attempts = Counter(e['point'] for e in trace['events'] if e['event'] == 'NAV_GOAL')
    rows = []
    for i, (p, requested) in enumerate(trace['points'], 1):
        row = dict.fromkeys(COLUMNS)
        status = 'CONFIRMED' if p in ledger else ('ATTEMPTED_INCOMPLETE' if p in bounds else
                 ('NOT_REACHED' if p in trace['selected'] else 'NOT_SELECTED'))
        row.update(ordinal=i, point_id=p, status=status, requested_dose=requested, ledger_dose=ledger.get(p))
        if p in bounds:
            start, end = bounds[p]
            counts = totals[p]
            if (sum(counts[k] for k in PHASES) != end - start
                or sum(counts[f'navigation_{k}_s'] for k in COMMANDS) != counts['travel_s']
                or sum(counts[f'body_{k}_during_nav_s'] for k in MOTION) != counts['travel_s']):
                raise ValueError('Per-point intervals do not reconcile')
            if p in ledger and (ledger[p] + 1e-9 < requested or counts['spray_flow_s'] <= 0):
                raise ValueError('Completed dose without quantity and independent physical discharge')
            row.update(start_sim_s=secs(start), end_sim_s=secs(end), elapsed_s=secs(end-start), nav_attempts=attempts[p])
            for key in COLUMNS[COLUMNS.index('travel_s'):COLUMNS.index('nav_attempts')]:
                row[key] = secs(counts[key])
        rows.append(row)
    def summary(selected):
        keys = ('elapsed_s', *COLUMNS[COLUMNS.index('travel_s'):COLUMNS.index('nav_attempts')])
        return {k: round(sum(row[k] for row in selected), 6) for k in keys}

    spans = list(bounds.values())
    return dict(schema='A1-PHASE-PROFILE/1', session=trace['session'], mission_version=trace['mission_version'],
                source_sha256=trace['source_sha256'], original_points=len(rows), selected_points=len(trace['selected']),
                status_counts=dict(Counter(r['status'] for r in rows)), start_before_first_goal_s=secs(spans[0][0]) if spans else None,
                cleanup_after_last_cycle_s=round(trace['cutoff_sim_s']-secs(spans[-1][1]), 6) if spans else None,
                end_sim_s=trace['cutoff_sim_s'], terminal_error=trace['terminal_error'], original_mission_passed=trace['mission_passed'],
                controller_collision_warnings_full_log=trace['controller_collision_warnings'], formal_credit=False,
                semantics={'clock': 'simulation seconds; interval [previous confirmation, next confirmation), first begins at first NAV_GOAL; last partial ends at stop request',
                    'travel_s': 'active global navigation including pauses inside the controller. Gate command modes and independently observed body motion subdivide the same travel total.',
                    'precision_s': 'navigation handoff through settled check, including local corrections; explicit localization holds and positive discharge take precedence.',
                    'localization_wait_s': 'explicit mission localization interruption through resume/revalidation, excluding startup; concurrent sensor rejection alone is not charged.',
                    'spray_flow_s': 'sampled independent physical flow > 0, left sample held to next capture; includes discharge during cancelled attempts. Dose is reconciled separately against SQLite.',
                    'other_s': 'tool arming/stopping, perception waits, handoff, residual retry outside observed settling and other unattributed protocol time.',
                    'motion': 'Physical velocity and yaw rate describe the preceding sample interval; command states and tool flow hold forward. Linear cutoff 0.015 m/s; angular cutoff 0.03 rad/s; gate command slow/medium/cruise cutoffs 0.10/0.25 m/s. Physical rotation estimated from consecutive sampled yaw; physical evaluations are offline only.',
                    'unobserved': 'null durations for unattempted points; warning count is for the entire original controller log, not a count of physical collisions.'},
                samples={k: {field:v for field,v in trace[k].items() if field!='changes'} for k in ('commands','body','flow')},
                confirmed_totals_s=summary([r for r in rows if r['status']=='CONFIRMED']),
                observed_totals_s=summary([r for r in rows if r['elapsed_s'] is not None]), rows=rows)


def history(reference, traces):
    reports = [reference] + traces
    ref_ids = {row['point_id'] for row in reference['rows'] if row['status'] == 'CONFIRMED'}
    ref_selected = [row['point_id'] for row in reference['rows'] if row['status'] != 'NOT_SELECTED']
    sessions = []
    for report in reports:
        rows = [row for row in report['rows'] if row['status'] == 'CONFIRMED']
        selected = [row['point_id'] for row in report['rows'] if row['status'] != 'NOT_SELECTED']
        totals = report['confirmed_totals_s']
        sessions.append(dict(session=report['session'], mission_version=report['mission_version'],
            selected=len(selected), confirmed=len(rows), attempted_incomplete=report['status_counts'].get('ATTEMPTED_INCOMPLETE', 0),
            not_reached=report['status_counts'].get('NOT_REACHED', 0), not_selected=report['status_counts'].get('NOT_SELECTED', 0),
            navigation_largest_on_confirmed_points=sum(
                r['travel_s'] > max(r[k] for k in PHASES if k != 'travel_s') for r in rows),
            confirmed_phase_totals_s={k: totals[k] for k in ('elapsed_s', *PHASES)},
            confirmed_navigation_fraction=round(totals['travel_s']/totals['elapsed_s'], 6) if totals['elapsed_s'] else None,
            common_confirmed_ids_with_reference=len(ref_ids.intersection(r['point_id'] for r in rows)),
            same_selection_as_reference=selected == ref_selected,
            same_mission_version_as_reference=report['mission_version'] == reference['mission_version'],
            collision_warning_lines=report['controller_collision_warnings_full_log'],
            raw_event_sha256=report['source_sha256']['mission_events.jsonl']))
    if len({s['session'] for s in sessions}) != len(sessions):
        raise ValueError('Duplicate sessions in history')
    return dict(schema='A1-PHASE-HISTORY/1', reference_session=reference['session'], sessions=sessions,
            interpretation='Each row uses the same five disjoint phases. Navigation must exceed every other phase on that specific confirmed point. These sessions include overlapping points and successive software versions; they are not independent trials of one release.')


def read_trace(path):
    return json.loads(gzip.decompress(path.read_bytes()))


def dump(report):
    return json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'


def csv_report(rows):
    text = io.StringIO(newline='')
    writer = csv.DictWriter(text, fieldnames=COLUMNS, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return text.getvalue()


def history_report(directory):
    paths = sorted((directory / 'phase_history').glob('*.json.gz'))
    if not paths:
        raise ValueError('No historical traces found')
    ref = analyze(read_trace(directory / DEFAULT_TRACE.name))
    return history(ref, [analyze(read_trace(path)) for path in paths])


def verify(directory):
    report = analyze(read_trace(directory / DEFAULT_TRACE.name))
    expected = {'point_phase_profile.json': dump(report), 'point_phase_profile.csv': csv_report(report['rows']),
                'phase_history.json': dump(history_report(directory))}
    for name, text in expected.items():
        if (directory / name).read_bytes() != text.encode():
            raise ValueError('Published replay differs: ' + name)
    print('Verified A1 per-point JSON, CSV and cross-session results from compact sensor observations.')


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    source = cli.add_mutually_exclusive_group()
    source.add_argument('--run', type=Path, help='Closed on-host raw A1 session')
    source.add_argument('--trace', type=Path, help='Published compact observation trace')
    source.add_argument('--history', action='store_true', help='Summarize all packaged sessions')
    source.add_argument('--verify', action='store_true', help='Replay and compare the published JSON, CSV and history')
    cli.add_argument('--evidence-dir', type=Path, default=DEFAULT_TRACE.parent)
    cli.add_argument('--save-trace', type=Path, help='Save deterministic lossless event/state projection (.json.gz)')
    cli.add_argument('--json', type=Path, help='Write full JSON report here; stdout by default')
    cli.add_argument('--csv', type=Path, help='Write per-point table, including unreached points')
    args = cli.parse_args()
    if args.verify:
        verify(args.evidence_dir)
        return
    if args.history:
        report = history_report(args.evidence_dir)
    else:
        trace = capture(args.run) if args.run else read_trace(args.trace or args.evidence_dir / DEFAULT_TRACE.name)
        report = analyze(trace)
        if args.save_trace:
            raw = json.dumps(trace, separators=(',', ':'), allow_nan=False).encode()
            args.save_trace.write_bytes(gzip.compress(raw, mtime=0))
    if args.json:
        args.json.write_text(dump(report))
    else:
        print(dump(report), end='')
    if args.csv:
        args.csv.write_bytes(csv_report(report['rows']).encode())


if __name__ == '__main__':
    main()
