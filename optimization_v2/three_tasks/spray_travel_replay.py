"""Compare the frozen candidate geometry to actual original-order leg measurements."""
import argparse
import json
import math
from pathlib import Path
from statistics import median

from spray_equal_length import angle, heading_change
from spray_route_time import MISSION, PROJECT, digest

PUBLISHED = PROJECT / 'validation/r10_bounded_20261119/a1'
HISTORY = PROJECT / 'validation/closeout_20261119/spray_time_budget.json'
PLAN = PROJECT / 'optimization_v2/three_tasks/candidates/A1_equal_length_turns.json'
PROFILE = PUBLISHED / 'point_phase_profile.json'
REPORT = PUBLISHED / 'travel_order_replay.json'


def read(path):
    return json.loads(path.read_text())


def describe(group):
    if not group:
        return {'n': 0}
    return {'n': len(group), 'median_nav_s': round(median(row['nav_s'] for row in group), 3),
            'minimum_nav_s': min(row['nav_s'] for row in group),
            'maximum_nav_s': max(row['nav_s'] for row in group),
            'slowest_point': max(group, key=lambda row: row['nav_s'])['id'],
            'median_body_rotate_s': (round(median(row['body_rotate_s'] for row in group), 3)
                                     if all(row['body_rotate_s'] is not None for row in group) else None)}


def build():
    mission, plan = read(MISSION), read(PLAN)
    history, latest = read(HISTORY), read(PROFILE)
    cluster = next(c for c in mission['clusters'] if c['id'] == 'A1')
    records = cluster['points']
    ids = [p['id'] for p in records]
    pts = [p['target'] for p in records]
    tests = []
    for run in history['sessions']:
        tests.append((run['session'], [{'id': r['id'], 'confirmed': r['confirmed'],
                       'started': r['first_nav_sim_s'] is not None,
                       'nav_s': r['phase_s']['navigation'], 'body_rotate_s': None} for r in run['point_cycles']]))
    tests.append((latest['session'], [{'id': r['point_id'], 'confirmed': r['status'] == 'CONFIRMED',
                  'started': r['status'] != 'NOT_REACHED', 'nav_s': r['travel_s'],
                  'body_rotate_s': r['body_rotating_during_nav_s']} for r in latest['rows']]))
    observed_edges = set()
    summaries = []
    for run, rows in tests:
        completed = {row['id'] for row in rows if row['confirmed']}
        observed_edges.update((a, b) for a, b in zip(ids, ids[1:]) if a in completed and b in completed)
        # Match 3 m translations of the original route. Keep diagonal, depot and
        # censored cases visible but exclude them from the straight / 90° comparison.
        points = []
        for row in rows:
            if not row['started']:
                continue
            i = ids.index(row['id'])
            d = math.dist(pts[i-1], pts[i]) if i else 0.
            yaw = angle(pts[i-2], pts[i-1]) if i >= 2 else cluster['start'][2]
            turn = math.degrees(heading_change(yaw, angle(pts[i-1], pts[i]))) if i else 0.
            group = ('straight' if math.isclose(turn, 0., abs_tol=1e-8) else
                     'quarter_turn' if math.isclose(turn, 90., abs_tol=1e-8) else 'other')
            points.append({**row, 'ordinal': i+1, 'length_m': round(d, 4),
                           'turn_deg': round(turn, 4), 'group': group})
        def compared(start):
            return {g: describe([p for p in points if p['confirmed'] and p['ordinal'] >= start
                                 and p['length_m'] == 3. and p['group'] == g])
                    for g in ('straight', 'quarter_turn')}
        summaries.append({'session': run, 'all_matched_3m_from_second_leg': compared(3),
                          'suffix_from_original_ordinal_14': compared(14),
                          'censored': [p for p in points if not p['confirmed']],
                          'confirmed_points': len(completed), 'selected_points': 56})
    order = plan['ordered_ids']
    edges = list(zip(order, order[1:]))
    direct = sum((a, b) in observed_edges for a,b in edges)
    reverse = sum((a,b) not in observed_edges and (b,a) in observed_edges for a,b in edges)
    return {'schema': 'A1-REDUCED-TURN-REPLAY/1', 'source_sha256': {
            str(f.relative_to(PROJECT)): digest(f) for f in (MISSION, PLAN, HISTORY, PROFILE)},
            'method': 'Only frozen prior, service points and drive limits are input to optimization. '
                      'Historical runs and the suffix of the latest run are observational checks. No time model is fitted.',
            'original': plan['original'], 'candidate': plan['candidate'], 'certificate': plan['certificate'],
            'proxy_reduction_s': round(plan['original']['translation_and_turn_proxy_s'] -
                                       plan['candidate']['translation_and_turn_proxy_s'], 6),
            'sample_medians': summaries,
            'coverage_of_new_legs': {'total': len(edges), 'same_direction_in_confirmed_original_runs': direct,
                                     'seen_only_in_reverse': reverse, 'neither_direction_seen': len(edges)-direct-reverse},
            'qualification': {'static_geometry_and_turn_objective': 'PASS', 'new_route_closed_loop': 'NOT_RUN',
                              'new_route_elapsed_sim_s': None, 'formal_cluster_credit': False},
            'limits': 'Recorded original-order segments carry safety stops, speed regulation and localization recovery. '
                      'The physical turn times describe original executions. A same-direction leg can have a different '
                      'starting pose and heading in the new task. Future pilot runs still use Nav2 collision checks.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', type=Path)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()
    result = build()
    if args.write:
        args.write.write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    if args.check and read(REPORT) != result:
        raise SystemExit('Published projection differs from the replay of committed inputs')
    print(json.dumps({k: result[k] for k in ('qualification','proxy_reduction_s','coverage_of_new_legs')}, indent=2))


if __name__ == '__main__':
    main()
