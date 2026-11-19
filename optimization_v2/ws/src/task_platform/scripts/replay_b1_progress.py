"""Replay entrance interruptions through both progress counters without a simulator."""
import argparse
import bisect
import gzip
import hashlib
import json
from pathlib import Path

from qualify_b1_progress import ContinuousRetryProgress
from qualify_b1_steering import RetryProgress
from groups_mowing_r10 import retry_exhausted


def load_trace(run):
    def read(name):
        return json.loads((run / name).read_text())

    def rows(name):
        with (run / name).open() as stream:
            return [json.loads(line) for line in stream if line.strip()]

    cfg = read('mission.json')
    spec = read('spec.json')
    segment = next(c for c in cfg['clusters'] if c['cluster'] == 'B1')['segments'][0]
    events = rows('mission_events.jsonl')
    result = read('mission_result.json')
    interruptions = []
    begin = None
    for record in events:
        if record['event'] == 'SEGMENT_GOAL' and record['segment'] == segment['id']:
            begin = record['t']
        elif record['event'] == 'BT_INTERRUPTS_SEGMENT' and begin is not None:
            interruptions.append([begin, record['t'], record['reason']])
            begin = None
    if not interruptions or result['completed_segments'] or spec['segment_start'] != 0:
        raise ValueError('Expected an interrupted entrance segment with no completed segment')
    gates = rows('control/gate.jsonl')
    times = [g['t'] for g in gates]
    refs = rows('reference/reference.jsonl')
    samples = []
    for ref in refs:
        k = bisect.bisect_right(times, ref['t']) - 1
        if k < 0 or ref['t'] < interruptions[0][0] or ref['t'] > interruptions[-1][1]:
            continue
        gate = gates[k]
        samples.append([ref['t'], {k:ref.get(k) for k in ('t','odom_t','valid','pose','odom','localization_epoch')},
                        {k:gate.get(k) for k in ('mode','allowed','armed_direction','t')}])
    inputs = ['mission.json','spec.json','mission_result.json','mission_events.jsonl','control/gate.jsonl','reference/reference.jsonl']
    return dict(schema=2, session=spec['session'], version=cfg['version'],
                source_sha256={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in inputs},
                terminal_error=result['error'], speed_cap_mps=cfg['speed_cap_mps'],
                segment={'id':segment['id'], 'direction':segment['direction'], 'poses':segment['poses']},
                interruptions=interruptions, samples=samples,
                # A START_WORK cancellation between two in-motion cancellations
                # also consumes an attempt in the recorded worker.
                other_interruptions=[[e['t'],e.get('reason')] for e in events if e['event']=='BT_INTERRUPTS_START_WORK'])


def evaluate(trace):
    if trace.get('schema') != 2:
        raise ValueError('Unknown schema')
    by_kind = {}
    for cls in (RetryProgress, ContinuousRetryProgress):
        progress = cls(trace['segment'], trace['interruptions'][0][0],trace['speed_cap_mps'])
        counters = 0
        previous_end = -1.
        results = []
        intervals = [(*r, True) for r in trace['interruptions']]
        intervals += [(t,t,reason,False) for t,reason in trace['other_interruptions']]
        for begin, end, reason, in_motion in sorted(intervals, key=lambda row:row[1]):
            # Match the worker tick: observe the odometry stream between actions,
            # credit arc only in navigation. Tool-start interruptions consume an
            # attempt but never invoke the in-motion rearm branch.
            for stamp, loc, gate in trace['samples']:
                if previous_end < stamp <= end:
                    if cls is ContinuousRetryProgress:
                        progress.observe(loc,gate,stamp,navigating=begin <= stamp < end)
                    elif begin <= stamp < end:
                        progress.observe(loc,gate)
            previous_end = end
            counters += 1
            rearmed = in_motion and progress.interrupted(end)
            if rearmed:
                counters = 0
            results.append(dict(end_sim_s=end, reason=reason, in_motion=in_motion, path_progress_m=round(progress.best,4),
                                resets_consecutive_counter=rearmed, consecutive=counters,
                                consecutive_bound_reached=retry_exhausted(counters,1,False)))
        by_kind[cls.__name__] = results
    return dict(session=trace['session'], version=trace['version'], terminal_error=trace['terminal_error'],
                source_sha256=trace['source_sha256'], counters=by_kind,
                scope='Same recorded online pose, odometry, gate and cancellations. A replayed rearm grants only another bounded attempt. No subsequent driving or area is inferred.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--run', type=Path)
    source.add_argument('--trace', type=Path)
    parser.add_argument('--save-trace',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    trace=load_trace(args.run) if args.run else json.loads(gzip.decompress(args.trace.read_bytes()))
    if args.save_trace:
        args.save_trace.write_bytes(gzip.compress(json.dumps(trace,separators=(',',':')).encode(),mtime=0))
    value=evaluate(trace)
    text=json.dumps(value,ensure_ascii=False,indent=2)+'\n'
    if args.output:
        args.output.write_text(text)
    else:
        print(text,end='')


if __name__=='__main__':
    main()
