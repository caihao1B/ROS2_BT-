"""Produce a read-only original-obligation review after one closed three-task simulation."""
import argparse
import hashlib
import json
import math
import struct
from pathlib import Path


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def review(run):
    spec = read(run / 'spec.json')
    if spec['task'] not in ('spray', 'mowing'):
        raise ValueError('Expected a point-service or mowing run')
    task = spec['task']
    mission = read(run / 'mission.json')
    result = read(run / 'mission_result.json')
    runtime = read(run / 'runtime.json')
    analysis_name = f'analysis_{task}_r10.json'
    analysis = read(run / analysis_name)
    if analysis['task_result'] != result:
        raise ValueError('Mission and analysis belong to different results')
    cluster = next(c for c in mission['clusters'] if c.get('id', c.get('cluster')) == spec['cluster'])
    version = mission['version']
    jobs = {}
    for job in analysis['jobs']:
        prefix, separator, ident = job['key'].partition(':')
        if prefix != version or not separator or ident in jobs:
            raise ValueError('Work key version or uniqueness failed')
        jobs[ident] = job
    rows = []
    if task == 'spray':
        points = cluster['points']
        completed = {e['point'] for e in result['completed']}
        for number, point in enumerate(points):
            key = point['id']
            requested = point['quantity']
            job = jobs.get(key, {})
            confirmed = job.get('confirmed', 0.)
            if key in completed and (job.get('state') != 'SUCCEEDED' or job.get('uncertain') is not False):
                raise ValueError('A completed point lacks its final ledger confirmation')
            state = ('CONFIRMED' if key in completed else 'CHECK_UNCERTAINTY' if job.get('uncertain')
                     else 'PARTIAL_FEEDBACK' if job else 'NO_RECORDED_SERVICE')
            rows.append(dict(key=key, state=state, selected=number < (spec['point_limit'] or len(points)),
                             target_map=point['target'], requested=requested, metered=confirmed,
                             nominal_remaining=max(0., requested-confirmed)))
        totals = dict(original_points=len(points), selected=result['selected_points'],
                      confirmed_points=len(completed), residual_points=len(points)-len(completed))
    else:
        segments = cluster['segments']
        selected = segments[spec.get('segment_start', 0):]
        if spec['segment_limit']:
            selected = selected[:spec['segment_limit']]
        arrivals = set(result['completed_segments'])
        for original in cluster.get('original_work_segments', [s for s in segments if s.get('tool_on')]):
            key = original['id']
            job = jobs.get(key, {})
            pieces = [s['id'] for s in segments if s.get('tool_on') and s.get('work_key', s['id']) == key]
            requested = original['requested_area_m2']
            # ToolTask transmits Point32 vertices. Reproduce its shoelace area before comparing.
            vertices = [struct.unpack('ff', struct.pack('ff', *xy)) for xy in original['target_area']]
            wire_area = abs(math.fsum(a[0]*b[1] - a[1]*b[0] for a,b in zip(vertices,vertices[1:]+vertices[:1]))) / 2
            if job and not math.isclose(wire_area, job['requested_m2'], abs_tol=1e-6):
                raise ValueError('Original requested work area differs from the ledger')
            confirmed = job.get('confirmed_m2', 0.)
            state = ('NO_SAFE_FRAGMENT_IN_THIS_PLAN' if not pieces else 'CHECK_UNCERTAINTY' if job.get('uncertain')
                     else 'CONFIRMED' if job.get('status')=='SUCCEEDED' else 'PARTIAL_FEEDBACK' if job
                     else 'NO_RECORDED_WORK')
            rows.append(dict(key=key, state=state, selected=any(p['id'] in pieces for p in selected),
                             planned_fragments=pieces, reached_fragments=[p for p in pieces if p in arrivals],
                             original_requested_m2=requested, ledger_requested_m2=job.get('requested_m2'), confirmed_for_key_m2=confirmed,
                             remaining_for_key_m2=max(0., job.get('requested_m2', requested)-confirmed)))
        totals = dict(original_cluster_m2=analysis['original_cluster_area_m2'],
                      selected_target_union_m2=analysis['selected_requested_area_m2'],
                      confirmed_union_m2=analysis['selected_confirmed_union_m2'],
                      residual_union_m2=analysis['selected_residual_m2'],
                      selected_segments=len(selected), reached_segments=len(arrivals),
                      untouched_reference_status=result.get('original_reference_status'))
    if set(jobs) - {r['key'] for r in rows}:
        raise ValueError('Ledger contains an unknown obligation')
    sources = ['spec.json','mission.json','mission_result.json',analysis_name,'runtime.json']
    if task == 'mowing':
        sources.append('coverage.geojson')
    digests = {}
    for name in sources:
        with (run / name).open('rb') as stream:
            digests[name] = hashlib.file_digest(stream, 'sha256').hexdigest()
    return dict(session=spec['session'], task=task, case=spec['case'], cluster=spec['cluster'],
                task_version=version, map_revision=mission['map_revision'], sources=digests,
                analysis_passed=analysis['passed'], clean_exit=runtime['passed'],
                summary=totals, obligations=rows, automatic_dispatch=False,
                scope='One closed run. Keep the original denominator. Per-key areas can overlap. Review localization, terrain, and access before scheduling remaining work.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path, help='Closed Gazebo result directory with the mission, analysis and ledger evidence.')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = review(args.run)
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        args.output.write_text(text+'\n', encoding='utf-8')
    else:
        print(text)
