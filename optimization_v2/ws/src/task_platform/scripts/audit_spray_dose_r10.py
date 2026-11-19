"""Cross-check a single spray job against the physical meter, effect evaluator and SQLite ledger."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sqlite3


def read_lines(file):
    with file.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def inspect(run, point):
    mission = json.loads((run / 'mission.json').read_bytes())
    tool = json.loads((run / 'scene/tool.json').read_bytes())
    key = mission['version'] + ':' + point
    database = run / 'ledger/ledger.sqlite'
    wal = Path(str(database) + '-wal')
    if wal.exists() and wal.stat().st_size > 0:
        raise ValueError('Use the sealed run after the ledger WAL has been checkpointed')
    db = sqlite3.connect(database.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    db.row_factory = sqlite3.Row
    try:
        job = dict(db.execute('SELECT key,quantity,status,reason,goal,uncertain,updated FROM jobs WHERE key=?', (key,)).fetchone())
        events = [dict(r) for r in db.execute('SELECT attempt,t,accepted,reason,quantity FROM events WHERE key=? ORDER BY t', (key,))]
        changes = [dict(r) for r in db.execute('SELECT t,status,reason FROM transitions WHERE key=? ORDER BY id', (key,))]
    finally:
        db.close()
    limit = json.loads(job.pop('goal'))['requested_quantity']
    receipts = [s for s in read_lines(run / 'mission_events.jsonl') if s['event'] == 'TOOL_RESULT']
    raw = [s for s in receipts if s.get('point') == point and not s.get('replayed')]
    next_receipt = next((s for s in receipts if s['t'] > raw[-1]['t'] and s['point'] != point), None)
    independent = json.loads((run / 'tool_evaluation/result.json').read_bytes())['jobs'][key]
    key_ops = defaultdict(list)
    for e in events:
        key_ops[e['attempt']].append(e)
    physical = defaultdict(list)
    integrals = defaultdict(float)
    active_count = Counter()
    previous = None
    for current in read_lines(run / 'tool_evaluation/physical.jsonl'):
        if current['key'] == key:
            attempt = current['attempt']
            physical[attempt].append(current)
            if abs(current['rpm_rad_s']) >= tool['working_min_rad_s']:
                active_count[attempt] += 1
                if previous and previous['key'] == key:
                    dt = current['t'] - previous['t']
                    if 0 < dt < .1:
                        integrals[attempt] += max(0., min(previous['flow'], current['flow'])) * dt
        previous = current
    assert len(raw) == len(key_ops) and len(changes) == 2 * len(raw)
    rows, last_quantity = [], 0.
    for i, (attempt, ledger_rows) in enumerate(key_ops.items()):
        receipt = raw[i]
        start, end = changes[2*i:2*i+2]
        samples = physical[attempt]
        last = ledger_rows[-1]['quantity']
        assert receipt['state'] == end['status'] and start['status'] == 'RUNNING'
        assert start['t'] <= ledger_rows[0]['t'] <= ledger_rows[-1]['t'] <= end['t'] <= receipt['t'] + 1.e-6
        assert math.isclose(last, receipt['confirmed'], abs_tol=1.e-9)
        assert math.isclose(max(0., limit-last), receipt['remaining'], abs_tol=1.e-9)
        assert samples[0]['flow'] == samples[-1]['flow'] == 0
        rows.append({'attempt': attempt, 'tool_receipt': receipt, 'transitions': [start, end],
                     'sqlite_evidence_count': len(ledger_rows), 'sqlite_reason_counts': dict(Counter(r['reason'] for r in ledger_rows)),
                     'accepted_observations': sum(e['accepted'] for e in ledger_rows),
                     'accepted_quantity_this_attempt': last-last_quantity, 'accepted_quantity_entire_key_at_end': last,
                     'command_feedback_meter_delta': receipt['meter_after']-receipt['meter_before'],
                     'physical_meter_first_last': [samples[0]['meter'], samples[-1]['meter']],
                     'physical_cumulative_meter_delta': samples[-1]['meter']-samples[0]['meter'],
                     'independent_integral_dose': integrals[attempt], 'independent_active_frames': active_count[attempt]})
        last_quantity = last
    assert math.isclose(sum(integrals.values()), independent['dose'], abs_tol=1.e-9)
    assert sum(active_count.values()) == independent['active_samples']
    assert math.isclose(last_quantity, job['quantity'], abs_tol=1.e-9)
    sources = ['ledger/ledger.sqlite', 'ledger/summary.json', 'tool_evaluation/result.json',
               'tool_evaluation/physical.jsonl', 'mission_events.jsonl', 'mission.json', 'scene/tool.json']
    summary = json.loads((run/'ledger/summary.json').read_bytes())
    final = next(j for j in summary['jobs'] if j['key'] == key)
    assert math.isclose(final['confirmed_quantity'], job['quantity'], abs_tol=1.e-9)
    return {'schema': 'R10-SINGLE-KEY-DOSE-AUDIT/1', 'session': tool['session'], 'point': point, 'key': key,
            'source_sha256': {n: hashlib.sha256((run/n).read_bytes()).hexdigest() for n in sources},
            'requested_quantity': limit, 'attempts': rows,
            'total': {'physical_meter_delta': sum(r['physical_cumulative_meter_delta'] for r in rows),
                      'observer_meter_delta': sum(r['command_feedback_meter_delta'] for r in rows),
                      'independent_evaluator_dose': independent['dose'], 'independent_evaluator_active_frames': independent['active_samples'],
                      's10_ledger_confirmed_key_quantity': job['quantity'], 'remaining_quantity': max(0., limit-job['quantity']),
                      'terminal': job['status'], 'uncertainty_pending': bool(job['uncertain'])},
            'following_point_first_receipt': next_receipt,
            'semantics': 'Physical meter is cumulative liquid flow irrespective of qualifying conditions. The independent 20 Hz evaluator integrates the smaller of consecutive flow readings while measured RPM qualifies. SQLite accepts only feedback observations passing the original docking, perception, stability, RPM, freshness, identity and continuity guards. The primary key persists across retries; action results report its cumulative qualified quantity. An incomplete point earns no completion credit. Subsequent point meter increments have a different key.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('point')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    data = json.dumps(inspect(args.run, args.point), indent=2, ensure_ascii=False) + '\n'
    if args.output:
        args.output.write_bytes(data.encode('utf-8'))
    else:
        print(data, end='')


if __name__ == '__main__':
    main()
